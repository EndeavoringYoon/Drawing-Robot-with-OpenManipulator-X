import sys, tty, termios, os
from typing import List, Dict, Tuple
from dynamixel_sdk import GroupSyncWrite,GroupSyncRead, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD, PacketHandler
from portclass import U2D2
import numpy as np
import platform
import glob
from id_maps import GET_ID_DICT_FUNC, GET_ID_LIST_FUNC
from transformation import *

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION  = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

ADDR_POS_P_GAIN            = 84
ADDR_POS_I_GAIN            = 82
ADDR_POS_D_GAIN            = 80

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

class RobotLauncher():
    def __init__(self,name):
        self.id_dict = GET_ID_DICT_FUNC[name]()
        self.ph = PacketHandler(2.0)
        self.u2d2_path = self.get_device_name()
        self.u2d2_list = []
        for i in range(len(self.u2d2_path)):
            u2d2 = U2D2(port=self.u2d2_path[i])
            self.u2d2_list.append(u2d2)
        self.id_to_bus = self.build_bus_index(self.u2d2_list)
        self.enable_torque()
        self.set_pid()
        self.smooth_transition_to_zero(joint_names=list(self.id_dict.keys()), duration=2.0, control_hz=50.0, verbose=True)
        # current_pos = self.get_current_pose(joint_names=list(self.id_dict.keys()), verbose=True)


    def is_tty(self):
        return os.isatty(sys.stdin.fileno())

    def getch(self):
        if not self.is_tty():
            print("getch() only works in terminal. Cannot be used in Jupyter or other IDE consoles.")
            return '\n'
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def get_device_name(self):
        """
        Dynamically determine the appropriate device name for the motor controller
        based on the operating system and available serial devices.

        Returns:
            str: The device name to use for the motor controller.
        """
        system = platform.system()

        if system == "Linux":
            # Check for USB devices
            usb_devices = glob.glob("/dev/ttyUSB*")
            if usb_devices:
                return usb_devices  # Use the first available USB device

            # If no USB devices, check for ACM devices
            acm_devices = glob.glob("/dev/ttyACM*")
            if acm_devices:
                return acm_devices[0]  # Use the first available ACM device

        elif system == "Darwin":  # macOS
            # Check for macOS USB serial devices
            try:
                mac_devices = glob.glob("/dev/tty.usbserial-*")
            except: 
                mac_devices = glob.glob("/dev/tty.usbmodemFFFFFFFEFFFF1")
            if mac_devices:
                return mac_devices  # Use the first available macOS USB serial device
        elif system == "Windows":
            import serial.tools.list_ports
            # Check for Windows COM ports
            ports = serial.tools.list_ports.comports()
            com_ports = [port.device for port in ports]
            if com_ports:
                return com_ports  # Use the first available COM port

        # If no devices are found, raise an error
        raise RuntimeError("No suitable device found for motor controller.")

    def enable_torque(self):
        for u2d2 in self.u2d2_list:
            for id in u2d2.ids:
                self.ph.write1ByteTxRx(u2d2.prt, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self):
        for u2d2 in self.u2d2_list:
            for id in u2d2.ids:
                self.ph.write1ByteTxRx(u2d2.prt, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_pid(self,pid=[800,100,100]):
        for u2d2 in self.u2d2_list:
            for id in u2d2.ids:
                self.ph.write2ByteTxRx(u2d2.prt, id, ADDR_POS_P_GAIN, pid[0])
                self.ph.write2ByteTxRx(u2d2.prt, id, ADDR_POS_I_GAIN, pid[1])
                self.ph.write2ByteTxRx(u2d2.prt, id, ADDR_POS_D_GAIN, pid[2])
    def set_specific_pid(self,pid=List[Tuple[int,int,int]],id_list:List[int]=[]):
        for u2d2 in self.u2d2_list:
            for id in u2d2.ids:
                if id in id_list:
                    idx = id_list.index(id)
                    self.ph.write2ByteTxRx(u2d2.prt, id, ADDR_POS_P_GAIN, pid[idx][0])
                    self.ph.write2ByteTxRx(u2d2.prt, id, ADDR_POS_I_GAIN, pid[idx][1])
                    self.ph.write2ByteTxRx(u2d2.prt, id, ADDR_POS_D_GAIN, pid[idx][2])

    def build_bus_index(self,u2d2_list: List) -> Dict[int, int]:
        """
        Create a reverse index (id -> bus_idx) found in all U2D2 instances.
        Assume that duplicate IDs will not exist simultaneously on different buses.
        """
        id_to_bus = {}
        for bus_idx, u in enumerate(u2d2_list):
            # u.ids: Support [(id, proto, model), ...] or [id, ...] format
            for entry in u.ids:
                dxl_id = entry[0] if isinstance(entry, (list, tuple)) else int(entry)
                id_to_bus[dxl_id] = bus_idx
        return id_to_bus

    @staticmethod
    def _pack_pos(val: float) -> bytes:
        """
        convert qpos(rad) to 4-byte little-endian ticks for Protocol 2.0 (Goal Position)
        -π..+π -> 0..4095 (0rad ≈ 2048)
        """
        # Reverse direction if needed: uncomment
        # val = -val

        # radian -> tick
        tick = int(round(4095 * ((val + np.pi) / (2 * np.pi))))

        # Range clipping: assuming joint mode (0..4095)
        tick = max(0, min(4095, tick))

        # 4-byte little-endian
        return bytes([
            DXL_LOBYTE(DXL_LOWORD(tick)),
            DXL_HIBYTE(DXL_LOWORD(tick)),
            DXL_LOBYTE(DXL_HIWORD(tick)),
            DXL_HIBYTE(DXL_HIWORD(tick)),
        ])
    
    @staticmethod
    def _unpack_pos(data: bytes) -> float:
        """
        convert 4-byte little-endian ticks to qpos(rad) for Protocol 2.0 (Present Position)
        0..4095 -> -π..+π (2048 ≈ 0rad)
        """
        # Extract tick from 4-byte little-endian
        tick = int.from_bytes(data, byteorder='little')
        
        # tick -> radian
        val = (tick / 4095.0) * (2 * np.pi) - np.pi
        
        return val

    def get_current_pose(self, joint_names: List[str], verbose: bool = False) -> List[float]:
        """
        Read current robot joint positions using GroupSyncRead per bus.
        """
        # 1. 각 버스별로 읽어야 할 id 목록 만들기
        bus_to_ids = {i: [] for i in range(len(self.u2d2_list))}
        name_to_id = {}
        name_to_bus = {}
        for name in joint_names:
            dxl_id = self.id_dict.get(name, None)
            bus_idx = self.id_to_bus.get(dxl_id, None)
            if dxl_id is not None and bus_idx is not None:
                bus_to_ids[bus_idx].append(dxl_id)
                name_to_id[name] = dxl_id
                name_to_bus[name] = bus_idx

        # 2. 각 버스별로 GroupSyncRead 실행
        id_to_pos = {}
        for bus_idx, id_list in bus_to_ids.items():
            if not id_list:
                continue
            u = self.u2d2_list[bus_idx]
            gsr = GroupSyncRead(u.prt, self.ph, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            for dxl_id in id_list:
                gsr.addParam(dxl_id)
            gsr.txRxPacket()
            for dxl_id in id_list:
                data = gsr.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                if data is not None:
                    pos_rad = self._unpack_pos(data.to_bytes(4, byteorder='little'))
                    id_to_pos[(bus_idx, dxl_id)] = pos_rad
                else:
                    if verbose:
                        print(f"[WARN] Data not available for ID {dxl_id} on bus {bus_idx}")
                    id_to_pos[(bus_idx, dxl_id)] = 0.0
            gsr.clearParam()

        # 3. joint_names 순서대로 결과 반환
        current_poses = []
        for name in joint_names:
            dxl_id = name_to_id.get(name, None)
            bus_idx = name_to_bus.get(name, None)
            if dxl_id is not None and bus_idx is not None:
                current_poses.append(id_to_pos.get((bus_idx, dxl_id), 0.0))
            else:
                if verbose:
                    print(f"[WARN] Unknown joint name or bus: {name}")
                current_poses.append(0.0)
        return current_poses

    def run(
        self,
        qpos: List[float],
        joint_names: List[str],
        verbose: bool = False,
    ):
        """
        Map qpos/body_names with same index to IDs and send GroupSyncWrite once per U2D2 bus.
        """
        if len(qpos) != len(joint_names):
            raise ValueError("qpos and joint_names length mismatch")

        # 2) Collect (id, packed_bytes) by bus
        per_bus_items: List[List[Tuple[int, List[int]]]] = [[] for _ in range(len(self.u2d2_list))]

        missing_name, missing_id, missing_bus = [], [], []

        for name, pos in zip(joint_names, qpos):
            dxl_id = self.id_dict.get(name, None)
            if dxl_id is None:
                missing_name.append(name)
                continue
            bus_idx = self.id_to_bus.get(dxl_id, None)
            if bus_idx is None:
                missing_id.append(dxl_id)
                continue
            per_bus_items[bus_idx].append((dxl_id, self._pack_pos(pos)))

        if verbose:
            if missing_name:
                print(f"[WARN] id missing for: {missing_name}")
            if missing_id:
                print(f"[WARN] id not found on any bus: {missing_id}")

        # 3) Send SyncWrite once per bus
        for bus_idx, items in enumerate(per_bus_items):
            if not items:
                continue
            u = self.u2d2_list[bus_idx]
            # Assume port is already open (opened during initialization)
            gsw = GroupSyncWrite(u.prt, self.ph, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
            for dxl_id, param in items:
                if not gsw.addParam(dxl_id, bytes(param)):
                    raise RuntimeError(f"GroupSyncWrite addParam failed for id {dxl_id} on bus {bus_idx}")
            # Send all at once
            dxl_comm_result = gsw.txPacket()
            if dxl_comm_result != 0:  # COMM_SUCCESS is 0
                raise RuntimeError(f"GroupSyncWrite txPacket failed on bus {bus_idx}, comm={dxl_comm_result}")
            gsw.clearParam()

        if verbose:
            for i, items in enumerate(per_bus_items):
                if items:
                    print(f"[BUS {i}] wrote {len(items)} motors.")
    
    def smooth_transition_to_zero(self, joint_names: List[str], duration: float = 3.0, 
                                 control_hz: float = 10.0, verbose: bool = False):
        """
        Smoothly transition from current pose to zero pose.
        
        Args:
            joint_names: List of joint names
            duration: Transition time (seconds)
            control_hz: Control frequency (Hz)
            verbose: Whether to show detailed output
        """
        import time
        
        # Read current pose
        current_poses = self.get_current_pose(joint_names, verbose=verbose)
        zero_poses = [0.0] * len(joint_names)
        
        if verbose:
            print(f"Transitioning from current pose to zero pose over {duration} seconds...")
            print(f"Current poses: {[f'{p:.3f}' for p in current_poses]}")
        
        # Calculate transition parameters
        dt = 1.0 / control_hz
        n_steps = int(duration * control_hz)
        
        if verbose:
            print(f"Control Hz: {control_hz}, Steps: {n_steps}, dt: {dt:.3f}s")
        
        # Execute smooth transition
        for step in range(n_steps + 1):
            alpha = step / n_steps  # 0.0 ~ 1.0
            
            # Calculate intermediate pose using linear interpolation
            interpolated_poses = []
            for current, target in zip(current_poses, zero_poses):
                interpolated_poses.append(current * (1 - alpha) + target * alpha)
            
            # Send command to robot
            self.run(joint_names=joint_names, qpos=interpolated_poses, verbose=False)
            
            # Timing control
            if step < n_steps:  # Do not wait on the last step
                time.sleep(dt)
        
        if verbose:
            print("Transition to zero pose completed!")
    
    def smooth_transition_to_pose(self, joint_names: List[str], target_poses: List[float], 
                                 duration: float = 3.0, control_hz: float = 10.0, verbose: bool = False):
        """
        Smoothly transition from current pose to target pose.
        
        Args:
            joint_names: List of joint names
            target_poses: Target joint positions (radians)
            duration: Transition time (seconds)
            control_hz: Control frequency (Hz)
            verbose: Whether to show detailed output
        """
        import time
        
        # Input validation
        if len(joint_names) != len(target_poses):
            raise ValueError(f"joint_names length ({len(joint_names)}) != target_poses length ({len(target_poses)})")
        
        # Read current pose
        current_poses = self.get_current_pose(joint_names, verbose=verbose)
        
        if verbose:
            print(f"Transitioning from current pose to target pose over {duration} seconds...")
            print(f"Current poses: {[f'{p:.3f}' for p in current_poses]}")
            print(f"Target poses: {[f'{p:.3f}' for p in target_poses]}")
        
        # Calculate transition parameters
        dt = 1.0 / control_hz
        n_steps = int(duration * control_hz)
        
        if verbose:
            print(f"Control Hz: {control_hz}, Steps: {n_steps}, dt: {dt:.3f}s")
        
        # Execute smooth transition
        for step in range(n_steps + 1):
            alpha = step / n_steps  # 0.0 ~ 1.0
            
            # Calculate intermediate pose using linear interpolation
            interpolated_poses = []
            for current, target in zip(current_poses, target_poses):
                interpolated_poses.append(current * (1 - alpha) + target * alpha)
            
            # Send command to robot
            self.run(joint_names=joint_names, qpos=interpolated_poses, verbose=False)
            
            # Timing control
            if step < n_steps:  # Do not wait on the last step
                time.sleep(dt)
        
        if verbose:
            print("Transition to target pose completed!")
    
    def smooth_transition_to_first_pose(self, joint_names: List[str], first_pose: List[float], 
                                       duration: float = 3.0, control_hz: float = 10.0, 
                                       verbose: bool = False):
        """
        Smoothly transition from current pose to first motion pose.
        
        Args:
            joint_names: List of joint names
            first_pose: First motion pose (radians)
            duration: Transition time (seconds)
            control_hz: Control frequency (Hz)
            verbose: Whether to show detailed output
        """
        import time
        
        # Check if joint count matches
        if len(joint_names) != len(first_pose):
            raise ValueError(f"joint_names length ({len(joint_names)}) != first_pose length ({len(first_pose)})")
        
        if verbose:
            print("Transitioning to first motion pose...")
            print(f"First pose: {[f'{p:.3f}' for p in first_pose]}")
        
        # Get current pose
        current_poses = self.get_current_pose(joint_names)
        if verbose:
            print(f"Current poses: {[f'{p:.3f}' for p in current_poses]}")
        
        # Calculate transition parameters
        dt = 1.0 / control_hz
        n_steps = int(duration * control_hz)
        
        if verbose:
            print(f"Control Hz: {control_hz}, Steps: {n_steps}, dt: {dt:.3f}s")
        
        # Execute smooth transition
        for step in range(n_steps + 1):
            alpha = step / n_steps  # 0.0 ~ 1.0
            
            # Calculate intermediate pose using linear interpolation
            interpolated_poses = []
            for current, target in zip(current_poses, first_pose):
                interpolated_poses.append(current * (1 - alpha) + target * alpha)
            
            # Send command to robot
            self.run(joint_names=joint_names, qpos=interpolated_poses, verbose=False)
            
            # Timing control
            if step < n_steps:  # Do not wait on the last step
                time.sleep(dt)
        
        if verbose:
            print("Transition to first motion pose completed!")
    
    def play_motion_trajectory(self, joint_names: List[str], motion_data: np.ndarray, 
                              target_hz: float = 100.0, verbose: bool = False):
        """
        Play motion trajectory with precise timing.
        
        Args:
            joint_names: List of joint names
            motion_data: Motion data (N x M array, N=frame count, M=joint count)
            target_hz: Target playback frequency (Hz)
            verbose: Whether to show detailed output
        """
        import time
        
        # Input validation
        if motion_data.shape[1] != len(joint_names):
            raise ValueError(f"motion_data columns ({motion_data.shape[1]}) != joint_names length ({len(joint_names)})")
        
        n_frames = motion_data.shape[0]
        dt = 1.0 / target_hz
        
        if verbose:
            print(f"Playing motion trajectory: {n_frames} frames at {target_hz}Hz")
            print(f"Duration: {n_frames * dt:.2f} seconds")
        
        # Variables for precise timing control
        start_time = time.perf_counter()
        frame_times = np.arange(n_frames) * dt
        
        try:
            for frame_idx in range(n_frames):
                # Calculate current time
                current_time = time.perf_counter() - start_time
                target_time = frame_times[frame_idx]
                
                # Send command to robot
                qpos = motion_data[frame_idx].tolist()
                self.run(joint_names=joint_names, qpos=qpos, verbose=False)
                
                # Calculate wait time until next frame
                next_target_time = frame_times[frame_idx + 1] if frame_idx < n_frames - 1 else frame_times[frame_idx] + dt
                sleep_time = next_target_time - current_time
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif verbose and frame_idx % 100 == 0:  # Warning every 100 frames
                    print(f"[WARN] Frame {frame_idx}: Running {abs(sleep_time)*1000:.1f}ms behind schedule")
                    
        except KeyboardInterrupt:
            if verbose:
                print("\nMotion playback interrupted by user")
            raise
        except Exception as e:
            if verbose:
                print(f"Error during motion playback: {e}")
            raise
        
        if verbose:
            total_time = time.perf_counter() - start_time
            print(f"Motion playback completed in {total_time:.2f} seconds")
    
    def play_motion_with_interpolation(self, joint_names: List[str], motion_data: np.ndarray,
                                     source_hz: float, target_hz: float, verbose: bool = False,
                                     zero_start: bool = False, transition_duration: float = 3.0):
        """
        Resample motion data to different frequency and play it.
        
        Args:
            joint_names: List of joint names
            motion_data: Original motion data
            source_hz: Original motion frequency (Hz)
            target_hz: Target playback frequency (Hz)
            verbose: Whether to show detailed output
            zero_start: Whether to start from zero pose
            transition_duration: Time to transition from zero to first pose (seconds)
        """
        import time
        from scipy.interpolate import interp1d
        
        # Generate time axis
        source_dt = 1.0 / source_hz
        source_times = np.arange(motion_data.shape[0]) * source_dt
        
        target_dt = 1.0 / target_hz
        target_times = np.arange(0, source_times[-1] + target_dt, target_dt)
        
        if verbose:
            print(f"Resampling motion from {source_hz}Hz to {target_hz}Hz")
            print(f"Original: {motion_data.shape[0]} frames, New: {len(target_times)} frames")
        
        # Interpolate for each joint
        interpolated_motion = np.zeros((len(target_times), motion_data.shape[1]))
        for joint_idx in range(motion_data.shape[1]):
            interp_func = interp1d(source_times, motion_data[:, joint_idx], 
                                 kind='linear', bounds_error=False, 
                                 fill_value=(motion_data[0, joint_idx], motion_data[-1, joint_idx]))
            interpolated_motion[:, joint_idx] = interp_func(target_times)
        
        # If zero start option is enabled
        if zero_start:
            if verbose:
                print(f"Starting from zero pose with {transition_duration:.1f}s transition")
            
            # Move to zero pose
            zero_pose = np.zeros(len(joint_names))
            self.run(joint_names, zero_pose)
            time.sleep(1.0)  # Wait 1 second
            
            # Smoothly transition from zero to first pose
            transition_frames = int(transition_duration * target_hz)
            start_time = time.perf_counter()
            next_t = start_time
            
            for i in range(transition_frames):
                alpha = i / (transition_frames - 1)  # From 0 to 1
                current_pose = (1 - alpha) * zero_pose + alpha * interpolated_motion[0]
                
                self.run(joint_names, current_pose)
                
                next_t += target_dt
                sleep_time = next_t - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
            if verbose:
                print("Transition completed, starting main motion...")
        
        # Play resampled motion
        self.play_motion_trajectory(joint_names, interpolated_motion, target_hz, verbose)

    def off(self,off_torque = False):
        if off_torque:
            self.disable_torque()
        for u2d2 in self.u2d2_list:
            u2d2.prt.closePort()

