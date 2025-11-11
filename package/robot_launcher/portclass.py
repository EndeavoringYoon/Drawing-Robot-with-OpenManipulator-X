from dynamixel_sdk import *
from typing import List,Tuple, Dict

ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4         # Data Byte Length
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 1000000
# 스캔 기본 파라미터
DEFAULT_BAUDRATES = [1000000] # [3000000, 2000000, 1000000, 57600, 115200]
DEFAULT_ID_RANGE = range(0, 254)  # 0(Broadcast)는 제외하는 게 일반적이지만, 여기선 그냥 스킵됨
PROTOCOL_VERSION = 2.0

class U2D2:
    def __init__(self, port: str, proto_version: float = 2.0):
        self.prt = PortHandler(port)
        if not self.prt.openPort():
            raise RuntimeError(f"Failed to open port: {port}")

        # 스캐닝 전에 굳이 고정 BAUDRATE로 세팅할 필요 없음(스캔 안에서 바꿀 거라면)
        # self.prt.setBaudRate(BAUDRATE)

        result = self.scan_dynamixel_bus()
        # ★ 정수 키로 접근해야 함
        self.ids = [rec[0] if isinstance(rec, (list, tuple)) else int(rec) for rec in result.get(BAUDRATE, [])]
        print(f"motor id list @ {BAUDRATE}: {self.ids}")
        self.prt.setBaudRate(BAUDRATE)
    
    def scan_dynamixel_bus(
        self,
        baudrates: List[int] = DEFAULT_BAUDRATES,
        id_range = DEFAULT_ID_RANGE,
        verbose: bool = True,
    ) -> Dict[int, List[Tuple[int, float, int]]]:

        results: Dict[int, List[Tuple[int, float, int]]] = {}
        seen_ids = set()
        print(f"Scan - baurdates: {baudrates}, id_range: {id_range}")
        if not self.prt.is_open:  # 필요시 체크
            self.prt.openPort()

        try:
            for baud in baudrates:
                # ★ 실제 포트 보드레이트 변경
                if not self.prt.setBaudRate(baud):
                    if verbose:
                        print(f"[WARN] Failed to set baudrate {baud}")
                    continue

                if verbose:
                    print(f"[INFO] Scanning baudrate {baud}...")

                found_this_baud: List[Tuple[int, float, int]] = []

                packet = PacketHandler(PROTOCOL_VERSION)

                for dxl_id in id_range:
                    if dxl_id in (0,) or dxl_id in seen_ids:
                        continue

                    try:
                        model_number, comm_result, dxl_error = packet.ping(self.prt, dxl_id)
                    except TypeError:
                        ping_ret = packet.ping(self.prt, dxl_id)
                        if isinstance(ping_ret, tuple) and len(ping_ret) == 2:
                            comm_result, dxl_error = ping_ret
                            model_number = -1
                        else:
                            continue

                    if comm_result == COMM_SUCCESS and dxl_error == 0:
                        found_this_baud.append((dxl_id, PROTOCOL_VERSION, int(model_number)))
                        seen_ids.add(dxl_id)
                        if verbose:
                            print(f"  - Found ID {dxl_id} (proto {PROTOCOL_VERSION}, model {model_number})")

                if found_this_baud:
                    results[baud] = found_this_baud

        finally:
            # 스캔만 하고 닫을지 유지할지는 용도에 따라 선택
            # self.prt.closePort()
            pass

        if verbose:
            if results:
                print("\n[SUMMARY]")
                for b, items in results.items():
                    print(f"  Baud {b}: {[(i, p, m) for (i, p, m) in items]}")
            else:
                print("[SUMMARY] No Dynamixel found.")

        return results

    
if __name__ == "__main__":
    pkh = PacketHandler(2.0)
    u2d2 = U2D2("/dev/ttyUSB0",pkh)