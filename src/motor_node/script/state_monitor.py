#!/usr/bin/env python3

import can
import time
import os

# ==========================================
# [설정] 시스템 파라미터
# ==========================================
CHANNEL = 'can0'
NODE_IDS = [2, 4]  # 602(Node 2), 604(Node 4) 리스트
REFRESH_RATE = 0.05 # 여러 노드이므로 응답 속도를 위해 약간 조절

GEAR_RATIO  = 10.0  # motor_node.cpp 의 STEER_RATIO 와 같아야 한다
ENCODER_RES = 131072.0
# ==========================================

def read_sdo_robust(bus, node_id, index, subindex):
    try:
        # 버스 비우기 (이전 노드의 잔여 데이터 삭제)
        while bus.recv(0): pass
        
        cmd = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        bus.send(can.Message(arbitration_id=0x600 + node_id, data=cmd, is_extended_id=False))
        
        start = time.time()
        while time.time() - start < 0.05: # 타임아웃 단축 (실시간성)
            resp = bus.recv(0.005)
            if resp and resp.arbitration_id == 0x580 + node_id:
                if (resp.data[1] + (resp.data[2] << 8) == index):
                    return int.from_bytes(resp.data[4:], byteorder='little', signed=True)
        return None
    except:
        return None

def get_status_string(sw):
    if sw is None: return "❌ OFF"
    if (sw >> 3) & 1:      return "🔴 FAULT"
    if not ((sw >> 4) & 1):return "⚠️ LOW PWR"
    if (sw >> 6) & 1:      return "🔒 DSBLD"
    if (sw >> 2) & 1:      return "🟢 RUN"
    return "⚪ READY"

def main():
    try:
        with can.Bus(channel=CHANNEL, interface='socketcan') as bus:
            while True:
                results = []
                for node_id in NODE_IDS:
                    # 데이터 수집
                    sw      = read_sdo_robust(bus, node_id, 0x6041, 0x00)
                    pos_raw = read_sdo_robust(bus, node_id, 0x6064, 0x00)
                    vel_raw = read_sdo_robust(bus, node_id, 0x606C, 0x00)
                    inputs  = read_sdo_robust(bus, node_id, 0x60FD, 0x00)
                    
                    # 계산
                    angle = (pos_raw / ENCODER_RES / GEAR_RATIO * 360.0) if pos_raw is not None else 0.0
                    rpm = (vel_raw / ENCODER_RES * 60 / GEAR_RATIO) if vel_raw is not None else 0.0
                    
                    results.append({
                        'id': node_id,
                        'status': get_status_string(sw),
                        'rpm': rpm,
                        'angle': angle,
                        'sensors': inputs if inputs is not None else 0
                    })

                # 화면 출력 (한 번에 업데이트)
                os.system('cls' if os.name == 'nt' else 'clear')
                print(f"┏━━━━┳━━━━━━━━━━━━┳━━━━━━━━━━━━┳━━━━━━━━━━━━┳━━━━━━━━━━━━━━━┓")
                print(f"┃ ID ┃   STATUS   ┃  RPM (속도) ┃  DEG (각도) ┃ L / R / H     ┃")
                print(f"┣━━━━╋━━━━━━━━━━━━╋━━━━━━━━━━━━╋━━━━━━━━━━━━╋━━━━━━━━━━━━━━━┫")
                
                for res in results:
                    s_neg  = "🔴" if (res['sensors'] >> 0) & 1 else "⚪"
                    s_pos  = "🔴" if (res['sensors'] >> 1) & 1 else "⚪"
                    s_home = "🔴" if (res['sensors'] >> 2) & 1 else "⚪"
                    
                    print(f"┃ {res['id']:02d} ┃ {res['status']:^10} ┃ {res['rpm']:>10.1f} ┃ {res['angle']:>10.1f} ┃  {s_neg}   {s_pos}   {s_home}  ┃")
                
                print(f"┗━━━━┻━━━━━━━━━━━━┻━━━━━━━━━━━━┻━━━━━━━━━━━━┻━━━━━━━━━━━━━━━┛")
                print(" CTRL+C를 누르면 종료합니다.")
                
                time.sleep(REFRESH_RATE)

    except KeyboardInterrupt:
        print("\n모니터링 종료.")

if __name__ == "__main__":
    main()
