# import serial
# import socket
# import asyncio
# from asyncua import Server, ua

# # --- [1. 설정 정보] ---
# COM_PORT = 'COM3'
# MY_IP = '0.0.0.0'
# YOLO_PORT = 6000
# OPC_UA_PORT = 40000

# state = {
#     "target_state": 1, 
#     "t_spd_main": 50, "t_spd_sort": 50, "t_spd_load": 50,
#     "t_flags": 0, "scan_result": 0
# }

# # --- [2. MCU 송신: PC -> MCU] ---
# def send_to_stm32(ser):
#     try:
#         packet = bytearray([
#             0xFE, state["target_state"], 
#             state["t_spd_main"], state["t_spd_sort"], state["t_spd_load"], 
#             state["t_flags"], state["scan_result"], 0xFF
#         ])
#         ser.write(packet)
#         print(f"\n📤 [PC -> MCU] 패킷 전송: {packet.hex().upper()}")
#         print(f"   └─ 명령상태: {state['target_state']} | 속도: {state['t_spd_main']}/{state['t_spd_sort']}/{state['t_spd_load']}")
#         print(f"   └─ 플래그(조립): {bin(state['t_flags'])} | 비전결과: {state['scan_result']}")
#     except Exception as e:
#         print(f"❌ UART 송신 에러: {e}")

# # --- [3. MCU 수신: MCU -> PC -> OPC-UA (비트 쪼개기 및 출력)] ---
# async def handle_mcu_report(ser, r):
#     print("✅ MCU 보고 수신 핸들러 가동 중...")
#     while True:
#         if ser.in_waiting >= 8:
#             raw = ser.read(8)
#             if raw[0] == 0xFE and raw[7] == 0xFF:
#                 # 데이터 파싱
#                 curr_state = raw[1]
#                 s1, s2, s3 = raw[2], raw[3], raw[4]
#                 dev = raw[5]
#                 floor = raw[6]

#                 # 상위 서버 업데이트
#                 await r['state'].write_value(curr_state)
#                 await r['s1'].write_value(s1)
#                 await r['s2'].write_value(s2)
#                 await r['s3'].write_value(s3)
#                 await r['floor'].write_value(floor)
                
#                 # 장치 상태 비트 분해 (Unpacking)
#                 is_busy = bool((dev >> 2) & 0x01)
#                 is_robot = bool((dev >> 5) & 0x01)
#                 is_done = bool((dev >> 6) & 0x01)

#                 await r['is_busy'].write_value(is_busy)
#                 await r['is_robot'].write_value(is_robot)
#                 await r['robot_done'].write_value(is_done)

#                 print(f"\n📥 [MCU -> PC] 보고 수신: {raw.hex().upper()}")
#                 print(f"   └─ 현재상태: {curr_state} | 현재위치: {floor}층")
#                 print(f"   └─ 장치비트: Busy({is_busy}), RobotWork({is_robot}), RobotDone({is_done})")
#         await asyncio.sleep(0.01)

# # --- [4. YOLO 수신] ---
# async def handle_yolo(ser):
#     loop = asyncio.get_running_loop()
#     server_soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     server_soc.bind((MY_IP, YOLO_PORT))
#     server_soc.listen(5)
#     server_soc.setblocking(False)
#     print(f"✅ YOLO 서버 대기 중 (Port: {YOLO_PORT})")

#     while True:
#         client, _ = await loop.sock_accept(server_soc)
#         try:
#             data = (await loop.sock_recv(client, 1024)).decode().strip()
#             print(data)
#             if data:
#                 parts = data.split(',')
#                 if len(parts) > 1:
#                     state["scan_result"] = int(parts[1])
#                     print(f"\n📸 [YOLO -> PC] 결과 수신: {state['scan_result']} (즉시 전파 시작)")
#                     send_to_stm32(ser)
                    
#                     # [수정] MCU가 Rising Edge(0->1)를 매번 인식할 수 있도록 0으로 리셋
#                     state["scan_result"] = 0 
#         except Exception as e:
#             print(f"❌ YOLO 처리 에러: {e}")
#         finally:
#             client.close()

# # --- [5. 메인 서버 및 명령 감시] ---
# async def main():
#     try:
#         ser = serial.Serial(COM_PORT, 115200, timeout=0.1)
#         print(f"✅ MCU 포트 연결 성공: {COM_PORT}")
#     except:
#         print(f"❌ {COM_PORT} 연결 실패"); return

#     ua_server = Server()
#     await ua_server.init()
#     ua_server.set_endpoint(f"opc.tcp://{MY_IP}:{OPC_UA_PORT}")
#     idx = await ua_server.register_namespace("SMART_FACTORY")

#     # [명령 노드: 40001~40008]
#     n_t_state = await ua_server.nodes.objects.add_variable(ua.NodeId(40001, idx), "TargetState", 1)
#     n_t_s1 = await ua_server.nodes.objects.add_variable(ua.NodeId(40002, idx), "TargetSpeedMain", 50)
#     n_t_s2 = await ua_server.nodes.objects.add_variable(ua.NodeId(40003, idx), "TargetSpeedSort", 50)
#     n_t_s3 = await ua_server.nodes.objects.add_variable(ua.NodeId(40004, idx), "TargetSpeedLoad", 50)
    
#     n_agv_s_arr = await ua_server.nodes.objects.add_variable(ua.NodeId(40005, idx), "AgvSortArrived", False)
#     n_agv_s_dep = await ua_server.nodes.objects.add_variable(ua.NodeId(40006, idx), "AgvSortDeparted", False)
#     n_agv_l_arr = await ua_server.nodes.objects.add_variable(ua.NodeId(40007, idx), "AgvLoadArrived", False)
#     n_agv_l_dep = await ua_server.nodes.objects.add_variable(ua.NodeId(40008, idx), "AgvLoadDeparted", False)

#     # [보고 노드: 50001~50008]
#     r_nodes = {
#         'state': await ua_server.nodes.objects.add_variable(ua.NodeId(50001, idx), "CurrentState", 0),
#         's1':    await ua_server.nodes.objects.add_variable(ua.NodeId(50002, idx), "CurrentSpeedMain", 0),
#         's2':    await ua_server.nodes.objects.add_variable(ua.NodeId(50003, idx), "CurrentSpeedSort", 0),
#         's3':    await ua_server.nodes.objects.add_variable(ua.NodeId(50004, idx), "CurrentSpeedLoad", 0),
#         'floor': await ua_server.nodes.objects.add_variable(ua.NodeId(50005, idx), "CurrentFloor", 0),
#         'is_busy':    await ua_server.nodes.objects.add_variable(ua.NodeId(50006, idx), "IsLiftMoving", False),
#         'is_robot':   await ua_server.nodes.objects.add_variable(ua.NodeId(50007, idx), "IsRobotWorking", False),
#         'robot_done': await ua_server.nodes.objects.add_variable(ua.NodeId(50008, idx), "IsRobotDone", False)
#     }

#     cmd_nodes = [n_t_state, n_t_s1, n_t_s2, n_t_s3, n_agv_s_arr, n_agv_s_dep, n_agv_l_arr, n_agv_l_dep]
#     for n in cmd_nodes: await n.set_writable()

#     asyncio.create_task(handle_yolo(ser))
#     asyncio.create_task(handle_mcu_report(ser, r_nodes))

#     async with ua_server:
#         print(f"🚀 Gateway 구동 중! (Port: {OPC_UA_PORT})")
#         print(f"   ▶ 명령: ns=1;i=40001~40008 | 보고: ns=1;i=50001~50008")
        
#         while True:
#             v_st = await n_t_state.read_value()
#             v_s1, v_s2, v_s3 = await n_t_s1.read_value(), await n_t_s2.read_value(), await n_t_s3.read_value()
            
#             # Boolean 개별 신호를 패킷용 Flags 바이트로 조립 (Packing)
#             f0 = await n_agv_s_arr.read_value()
#             f1 = await n_agv_s_dep.read_value()
#             f2 = await n_agv_l_arr.read_value()
#             f3 = await n_agv_l_dep.read_value()
#             new_flags = (int(f0) << 0) | (int(f1) << 1) | (int(f2) << 2) | (int(f3) << 3)

#             # 변경 사항이 있을 때만 전송
#             if (v_st != state["target_state"] or new_flags != state["t_flags"] or 
#                 v_s1 != state["t_spd_main"] or v_s2 != state["t_spd_sort"] or v_s3 != state["t_spd_load"]):
                
#                 print(f"\n🔔 [OPC -> PC] 상위 서버 명령 감지됨")
#                 state.update({"target_state": v_st, "t_flags": new_flags, "t_spd_main": v_s1, "t_spd_sort": v_s2, "t_spd_load": v_s3})
#                 send_to_stm32(ser)
            
#             await asyncio.sleep(0.1)

# if __name__ == "__main__":
#     asyncio.run(main())

import serial
import socket
import asyncio
from asyncua import Server, ua

# --- [1. 설정 정보] ---
COM_PORT = 'COM3'
MY_IP = '0.0.0.0'
YOLO_PORT = 6000
OPC_UA_PORT = 40000

# 서버 내부 상태 관리
state = {
    "target_state": 1, 
    "t_spd_main": 50, "t_spd_sort": 50, "t_spd_load": 50,
    "t_flags": 0, "scan_result": 0
}

# --- [2. MCU 송신 로직] ---
def send_to_stm32(ser):
    try:
        packet = bytearray([
            0xFE, 
            int(state["target_state"]) & 0xFF, 
            int(state["t_spd_main"]) & 0xFF, 
            int(state["t_spd_sort"]) & 0xFF, 
            int(state["t_spd_load"]) & 0xFF, 
            state["t_flags"] & 0xFF, 
            state["scan_result"] & 0xFF, 
            0xFF
        ])
        ser.write(packet)
        print(f"📤 [PC -> MCU] 패킷 전송: {packet.hex().upper()}")
    except Exception as e:
        print(f"❌ UART 송신 에러: {e}")

# --- [3. MCU 수신 보고 핸들러] ---
async def handle_mcu_report(ser, r):
    while True:
        if ser.in_waiting >= 8:
            raw = ser.read(8)
            if raw[0] == 0xFE and raw[7] == 0xFF:
                # 데이터를 파싱하여 보고용 노드(50001~50008)에 업데이트
                await r['state'].write_value(ua.Variant(raw[1], ua.VariantType.Int64))
                await r['s1'].write_value(ua.Variant(raw[2], ua.VariantType.Int64))
                await r['s2'].write_value(ua.Variant(raw[3], ua.VariantType.Int64))
                await r['s3'].write_value(ua.Variant(raw[4], ua.VariantType.Int64))
                await r['floor'].write_value(ua.Variant(raw[6], ua.VariantType.Int64))
                
                # 비트 분해 (Busy, Robot 등)
                dev = raw[5]
                await r['is_busy'].write_value(bool((dev >> 2) & 0x01))
                await r['is_robot'].write_value(bool((dev >> 5) & 0x01))
                await r['robot_done'].write_value(bool((dev >> 6) & 0x01))
        await asyncio.sleep(0.01)

# --- [4. 메인 서버 및 노드 생성] ---
async def main():
    try:
        ser = serial.Serial(COM_PORT, 115200, timeout=0.1)
        print(f"✅ MCU 연결 성공: {COM_PORT}")
    except:
        print(f"❌ {COM_PORT} 연결 실패"); return

    ua_server = Server()
    await ua_server.init()
    ua_server.set_endpoint(f"opc.tcp://{MY_IP}:{OPC_UA_PORT}")
    ua_server.set_server_name("STM32_Gateway")

    # 보안 정책 (NoSecurity)
    ua_server.set_security_policy([ua.SecurityPolicyType.NoSecurity])

    # 네임스페이스 등록 (반드시 ns=2가 되도록 확인)
    idx = await ua_server.register_namespace("SMART_FACTORY")
    print(f"📌 Namespace Index = {idx}") 

    objects = ua_server.nodes.objects

    # --- [명령 노드: 40001 ~ 40008] ---
    # C# 클라이언트의 LLong 타입에 대응하기 위해 Int64 사용
    n_t_state = await objects.add_variable(ua.NodeId(40001, idx), "TargetState", ua.Variant(1, ua.VariantType.Int64))
    n_t_s1 = await objects.add_variable(ua.NodeId(40002, idx), "TargetSpeedMain", ua.Variant(50, ua.VariantType.Int64))
    n_t_s2 = await objects.add_variable(ua.NodeId(40003, idx), "TargetSpeedSort", ua.Variant(50, ua.VariantType.Int64))
    n_t_s3 = await objects.add_variable(ua.NodeId(40004, idx), "TargetSpeedLoad", ua.Variant(50, ua.VariantType.Int64))
    
    n_agv_s_arr = await objects.add_variable(ua.NodeId(40005, idx), "AgvSortArrived", False)
    n_agv_s_dep = await objects.add_variable(ua.NodeId(40006, idx), "AgvSortDeparted", False)
    n_agv_l_arr = await objects.add_variable(ua.NodeId(40007, idx), "AgvLoadArrived", False)
    n_agv_l_dep = await objects.add_variable(ua.NodeId(40008, idx), "AgvLoadDeparted", False)

    # 모든 명령 노드 쓰기 권한 부여
    cmd_nodes = [n_t_state, n_t_s1, n_t_s2, n_t_s3, n_agv_s_arr, n_agv_s_dep, n_agv_l_arr, n_agv_l_dep]
    for n in cmd_nodes: await n.set_writable()

    # --- [보고 노드: 50001 ~ 50008] ---
    r_nodes = {
        'state': await objects.add_variable(ua.NodeId(50001, idx), "CurrentState", ua.Variant(0, ua.VariantType.Int64)),
        's1':    await objects.add_variable(ua.NodeId(50002, idx), "CurrentSpeedMain", ua.Variant(0, ua.VariantType.Int64)),
        's2':    await objects.add_variable(ua.NodeId(50003, idx), "CurrentSpeedSort", ua.Variant(0, ua.VariantType.Int64)),
        's3':    await objects.add_variable(ua.NodeId(50004, idx), "CurrentSpeedLoad", ua.Variant(0, ua.VariantType.Int64)),
        'floor': await objects.add_variable(ua.NodeId(50005, idx), "CurrentFloor", ua.Variant(0, ua.VariantType.Int64)),
        'is_busy':    await objects.add_variable(ua.NodeId(50006, idx), "IsLiftMoving", False),
        'is_robot':   await objects.add_variable(ua.NodeId(50007, idx), "IsRobotWorking", False),
        'robot_done': await objects.add_variable(ua.NodeId(50008, idx), "IsRobotDone", False)
    }

    # 백그라운드 태스크 시작
    asyncio.create_task(handle_mcu_report(ser, r_nodes))

    async with ua_server:
        print(f"🚀 OPC UA 서버 가동 중 (ns={idx}, Port: {OPC_UA_PORT})")
        while True:
            # 1. 상위 서버에서 쓴 명령값 읽기
            v_st = await n_t_state.read_value()
            v_s1, v_s2, v_s3 = await n_t_s1.read_value(), await n_t_s2.read_value(), await n_t_s3.read_value()
            
            # 2. Boolean 플래그 조립
            f0, f1 = await n_agv_s_arr.read_value(), await n_agv_s_dep.read_value()
            f2, f3 = await n_agv_l_arr.read_value(), await n_agv_l_dep.read_value()
            new_flags = (int(f0) << 0) | (int(f1) << 1) | (int(f2) << 2) | (int(f3) << 3)

            # 3. 변경 사항이 있을 때만 STM32로 전송
            if (v_st != state["target_state"] or new_flags != state["t_flags"] or 
                v_s1 != state["t_spd_main"] or v_s2 != state["t_spd_sort"] or v_s3 != state["t_spd_load"]):
                
                print(f"🔔 명령 변경 감지 -> STM32 전송 시도")
                state.update({"target_state": v_st, "t_flags": new_flags, "t_spd_main": v_s1, "t_spd_sort": v_s2, "t_spd_load": v_s3})
                send_to_stm32(ser)

            await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())