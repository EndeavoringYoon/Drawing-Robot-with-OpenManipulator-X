#!/usr/bin/env python
import numpy as np
import sys, tty, termios,os
import platform
import glob

def is_tty():
    return os.isatty(sys.stdin.fileno())

def getch():
    if not is_tty():
        print("getch()는 터미널에서만 작동해요. Jupyter나 기타 IDE 콘솔에서는 쓸 수 없습니다.")
        return '\n'
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class XM430():
    def __init__(self,init=True):
        """----------INITIAL INSTANCES----------"""
        # Control table address

        ADDR_TORQUE_ENABLE          = 64
        ADDR_GOAL_POSITION          = 116
        ADDR_POS_P_GAIN            = 84
        ADDR_POS_I_GAIN            = 82
        ADDR_POS_D_GAIN            = 80
        LEN_GOAL_POSITION           = 4         # Data Byte Length
        ADDR_PRESENT_POSITION       = 132
        LEN_PRESENT_POSITION        = 4         # Data Byte Length
        DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 1000000
        PID=[[800,100,100],
             [800,100,100],
             [800,100,100],
             [800,100,100],
             [800,0,4700]] # p,i,d gain for each motor
        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        PROTOCOL_VERSION            = 2.0

        # Make sure that each DYNAMIXEL ID should have unique ID.
        DXL_ID                     = [11,12,13,14,15]               # Dynamixel#1 ID : 1

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        DEVICENAME                = '/dev/ttyUSB0'

        TORQUE_ENABLE               = 1                 # Value for enabling the torque
        TORQUE_DISABLE              = 0                 # Value for disabling the torque
        DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
        DXL_MAXIMUM_THRESHOLD        = 4000             # If one position step value exceeds this number, it stops
        DXL_MAXIMUM_STEP            = 50                # If one position step value exceeds this number, it will be devided into several movements
        self.PROTOCOL_VERSION=PROTOCOL_VERSION
        self.DEVICENAME=self.get_device_name()
        self.ADDR_GOAL_POSITION=ADDR_GOAL_POSITION
        self.LEN_GOAL_POSITION=LEN_GOAL_POSITION
        self.ADDR_PRESENT_POSITION=ADDR_PRESENT_POSITION
        self.LEN_PRESENT_POSITION=LEN_PRESENT_POSITION
        self.ADDR_TORQUE_ENABLE=ADDR_TORQUE_ENABLE
        self.TORQUE_ENABLE=TORQUE_ENABLE
        self.BAUDRATE=BAUDRATE
        self.DXL_ID=DXL_ID
        self.DXL_MOVING_STATUS_THRESHOLD=DXL_MOVING_STATUS_THRESHOLD
        self.ADDR_POS_P_GAIN=ADDR_POS_P_GAIN
        self.ADDR_POS_I_GAIN=ADDR_POS_I_GAIN
        self.ADDR_POS_D_GAIN=ADDR_POS_D_GAIN
        self.PID=PID
        #for while
        self.DXL_MINIMUM_POSITION_VALUE=DXL_MINIMUM_POSITION_VALUE
        self.DXL_MAXIMUM_POSITION_VALUE=DXL_MAXIMUM_POSITION_VALUE
        self.TORQUE_DISABLE=TORQUE_DISABLE
        self.DXL_MAXIMUM_THRESHOLD=DXL_MAXIMUM_THRESHOLD
        self.DXL_MAXIMUM_STEP=DXL_MAXIMUM_STEP



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
                return usb_devices[0]  # Use the first available USB device

            # If no USB devices, check for ACM devices
            acm_devices = glob.glob("/dev/ttyACM*")
            if acm_devices:
                return acm_devices[0]  # Use the first available ACM device

        elif system == "Darwin":  # macOS
            # Check for macOS USB serial devices
            mac_devices = glob.glob("/dev/tty.usbserial-*")
            if mac_devices:
                return mac_devices[0]  # Use the first available macOS USB serial device
        elif system == "Windows":
            import serial.tools.list_ports
            # Check for Windows COM ports
            ports = serial.tools.list_ports.comports()
            com_ports = [port.device for port in ports]
            if com_ports:
                return com_ports[0]  # Use the first available COM port

        # If no devices are found, raise an error
        raise RuntimeError("No suitable device found for motor controller.")

    
        
    def initialize(self): # motor initialization
        DEVICENAME=self.DEVICENAME
        ADDR_TORQUE_ENABLE=self.ADDR_TORQUE_ENABLE
        TORQUE_ENABLE=self.TORQUE_ENABLE
        BAUDRATE=self.BAUDRATE
        DXL_ID=self.DXL_ID


        # Initialize PortHandler instance
        port_handler = PortHandler(DEVICENAME)
        self.port_handler=port_handler
        
        # Initialize PacketHandler instance
        ph=Protocol2PacketHandler()
        self.ph=ph

        # Open port
        if port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if port_handler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()


        for i in range(0,len(DXL_ID)):
            # Enable Dynamixel Torque in U2D2_1
            dxl_comm_result, dxl_error = ph.write1ByteTxRx(port_handler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % ph.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % ph.getRxPacketError(dxl_error))
            # else:
            #     print("Dynamixel#%d has been successfully connected" % DXL_ID[i])
        self.set_parameter()
        self.set_pid()

    def set_pid(self):
        PID=self.PID
        DXL_ID=self.DXL_ID
        ADDR_POS_P_GAIN=self.ADDR_POS_P_GAIN
        ADDR_POS_I_GAIN=self.ADDR_POS_I_GAIN
        ADDR_POS_D_GAIN=self.ADDR_POS_D_GAIN
        ph=self.ph
        port_handler=self.port_handler
        for i in range(0,len(DXL_ID)):
            # Set PID Gain
            dxl_comm_result, dxl_error = ph.write2ByteTxRx(port_handler, DXL_ID[i], ADDR_POS_P_GAIN, PID[i][0])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % ph.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % ph.getRxPacketError(dxl_error))

            dxl_comm_result, dxl_error = ph.write2ByteTxRx(port_handler, DXL_ID[i], ADDR_POS_I_GAIN, PID[i][1])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % ph.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % ph.getRxPacketError(dxl_error))

            dxl_comm_result, dxl_error = ph.write2ByteTxRx(port_handler, DXL_ID[i], ADDR_POS_D_GAIN, PID[i][2])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % ph.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % ph.getRxPacketError(dxl_error))



    def set_parameter(self):
        DXL_ID=self.DXL_ID
        ADDR_PRESENT_POSITION=self.ADDR_PRESENT_POSITION
        ADDR_GOAL_POSITION=self.ADDR_GOAL_POSITION
        ph=self.ph

        #setting U2D2-1 txpacket data for read
        param_read=DXL_ID
        read_start_address=ADDR_PRESENT_POSITION
        param_length_read=len(param_read)
        data_length=4 # LOBYTE(LOWORD) HIBYTE(LOWORD) LOBYTE(HIWORD) HIBYTE(HIWORD)
        txpacket_read=[0]*(param_length_read + 14)
        txpacket_read[PKT_HEADER0] = 0xFF
        txpacket_read[PKT_HEADER1] = 0xFF
        txpacket_read[PKT_HEADER2] = 0xFD
        txpacket_read[PKT_RESERVED] = 0x00
        txpacket_read[4] = 0xFE
        txpacket_read[5] = DXL_LOBYTE(param_length_read + 7)  # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket_read[6] = DXL_HIBYTE(param_length_read + 7)  # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket_read[7] = 130 #INST_SYNC_READ - 130
        txpacket_read[8 + 0] = DXL_LOBYTE(read_start_address)
        txpacket_read[8 + 1] = DXL_HIBYTE(read_start_address)
        txpacket_read[8 + 2] = DXL_LOBYTE(data_length)
        txpacket_read[8 + 3] = DXL_HIBYTE(data_length)
        txpacket_read[8 + 4: 8 + 4 + param_length_read] = param_read[0: param_length_read]
        # total_packet_read_length = DXL_MAKEWORD(txpacket_read[5], txpacket_read[6]) + 7
        total_packet_read_length_1 = param_length_read +7 +7
        crc = ph.updateCRC(0, txpacket_read, total_packet_read_length_1 - 2)  # 2: CRC16
        txpacket_read[total_packet_read_length_1 - 2] = DXL_LOBYTE(crc)
        txpacket_read[total_packet_read_length_1 - 1] = DXL_HIBYTE(crc)
        self.txpacket_read=txpacket_read

        # setting U2D2-1 txpacket data for write
        write_start_address=ADDR_GOAL_POSITION
        param_length_write = len(DXL_ID) * (1 + data_length)
        txpacket_write = [0] * (param_length_write + 14)
        txpacket_write[4] = 0xFE
        txpacket_write[5] = DXL_LOBYTE(param_length_write + 7)  # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket_write[6] = DXL_HIBYTE(param_length_write + 7)  # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket_write[7] = 131
        txpacket_write[8 + 0] = DXL_LOBYTE(write_start_address)
        txpacket_write[8 + 1] = DXL_HIBYTE(write_start_address)
        txpacket_write[8 + 2] = DXL_LOBYTE(data_length)
        txpacket_write[8 + 3] = DXL_HIBYTE(data_length)
        self.txpacket_write=txpacket_write
        self.param_length_write = param_length_write
    def build_sync_packet(inst, start_address, data_length, dxl_ids, ph):
        param_length = len(dxl_ids) * (1 + (0 if inst == 131 else data_length))
        txpacket = [0] * (param_length + 14)

        txpacket[0:4] = [0xFF, 0xFF, 0xFD, 0x00]
        txpacket[4] = 0xFE
        txpacket[5] = DXL_LOBYTE(param_length + 7)
        txpacket[6] = DXL_HIBYTE(param_length + 7)
        txpacket[7] = inst
        txpacket[8] = DXL_LOBYTE(start_address)
        txpacket[9] = DXL_HIBYTE(start_address)
        txpacket[10] = DXL_LOBYTE(data_length)
        txpacket[11] = DXL_HIBYTE(data_length)

        if inst == 130:
            txpacket[12:12+len(dxl_ids)] = dxl_ids
        else:
            pass  # 쓰기 파라미터는 나중에 채워짐

        crc = ph.updateCRC(0, txpacket, len(txpacket) - 2)
        txpacket[-2] = DXL_LOBYTE(crc)
        txpacket[-1] = DXL_HIBYTE(crc)

        return txpacket, param_length
    
    def _set_parameter(self):
        ph = self.ph
        DXL_ID = self.DXL_ID
        data_length = 4

        # Read Packet
        self.txpacket_read, _ = self.build_sync_packet(
            inst=130,
            start_address=self.ADDR_PRESENT_POSITION,
            data_length=data_length,
            dxl_ids=DXL_ID,
            ph=ph
        )

        # Write Packet (파라미터만 미리 세팅)
        txpacket_write, param_length_write = self.build_sync_packet(
            inst=131,
            start_address=self.ADDR_GOAL_POSITION,
            data_length=data_length,
            dxl_ids=DXL_ID,
            ph=ph
        )

        self.txpacket_write = txpacket_write
        self.param_length_write = param_length_write

    def read(self):
        DXL_ID=self.DXL_ID
        port_handler=self.port_handler
        txpacket_read=self.txpacket_read
        _ = port_handler.writePort(txpacket_read)
        wait_length=15*len(DXL_ID)
        rx_length=0 # actually, we may not need this
        rxpacket=[]
        while True:
            rxpacket.extend(port_handler.readPort(wait_length-rx_length))
            if len(rxpacket)>=wait_length:
                if len(DXL_ID)!=0:
                    packet_result = np.array(rxpacket).reshape(len(DXL_ID),-1)[:,[4,9,10,11,12]]
                    dxl_present_position = ((packet_result[:,1] & 0xFF) | ((packet_result[:,2] & 0xFF) << 8) & 0xFFFF) | ((packet_result[:,3] & 0xFF) | ((packet_result[:,4] & 0xFF) << 8) & 0xFFFF) << 16
                else:
                    dxl_present_position = []
                break
        port_handler.is_using=False

        self.dxl_present_position=dxl_present_position
    
    def run_multi_motor(self, desired_pos,id_to_move):
        DXL_ID=self.DXL_ID
        ph=self.ph
        txpacket_write=self.txpacket_write
        port_handler = self.port_handler

        desired_pos=np.array(desired_pos).reshape(-1,1)
        desired_byte_pos= [(desired_pos & 0xFFFF)& 0xFF, ((desired_pos & 0xFFFF)>>8) & 0xFF, ((desired_pos >> 16) & 0xFFFF) & 0xFF, (((desired_pos >> 16) & 0xFFFF)>8) & 0xFF]
        desired_result = np.array([arr[:,0] for arr in np.array(desired_byte_pos)]).T.tolist()
        param_write=[]
        for i,motor_id in enumerate(id_to_move):
            if motor_id in DXL_ID:
                param_write.append(motor_id)
                param_write.extend(desired_result[i])

        txpacket_write[8 + 4: 8 + 4 + self.param_length_write] = param_write[0: self.param_length_write]
        _ = ph.txPacket(port_handler,txpacket_write)
        port_handler.is_using=False 


    def safety(self,current_pos,desired_pos,id_to_move):
        DXL_MAXIMUM_THRESHOLD=self.DXL_MAXIMUM_THRESHOLD
        DXL_MAXIMUM_STEP = self.DXL_MAXIMUM_STEP
        DXL_MOVING_STATUS_THRESHOLD=self.DXL_MOVING_STATUS_THRESHOLD
        DXL_ID = self.DXL_ID
        diff=np.zeros_like(current_pos)
        for i,n in enumerate(id_to_move):
            if n in DXL_ID:
                index = DXL_ID.index(n)
                diff[index]=desired_pos[i]-current_pos[index]

        max_diff=np.max(np.abs(diff))

        if (max_diff >= DXL_MAXIMUM_THRESHOLD): # exceed threshold -> stop
            print("It has exceeded the THRESHOLD!",max_diff)
            self.close()
            quit()
        elif(DXL_MAXIMUM_STEP <= max_diff < DXL_MAXIMUM_THRESHOLD): # disassemble desired poses into small steps
            print("devided movement")
            for i in range(1,round(max_diff/DXL_MAXIMUM_STEP)+1):
                self.run_multi_motor((np.array(current_pos)+i*diff*DXL_MAXIMUM_STEP/max_diff).astype(int).tolist(),DXL_ID)

        elif(DXL_MOVING_STATUS_THRESHOLD <= max_diff < DXL_MAXIMUM_STEP): #appropriate to move
            print("just go")
            self.run_multi_motor(current_pos+diff,DXL_ID)
        else: # to small to go
            print("too small")
            pass

    def close(self):
        DXL_ID=self.DXL_ID
        ADDR_TORQUE_ENABLE=self.ADDR_TORQUE_ENABLE
        TORQUE_DISABLE=self.TORQUE_DISABLE
        port_handler=self.port_handler
        ph=self.ph
        for i in range(0,len(DXL_ID)):
            # Disable Dynamixel#1 Torque
            dxl_comm_result, dxl_error = ph.write1ByteTxRx(port_handler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % ph.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % ph.getRxPacketError(dxl_error))

        # Close port
        port_handler.closePort()
        try:
            quit()
        except:
            pass
        
    
