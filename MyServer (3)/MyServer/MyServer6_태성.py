import serial
import socket
import asyncio
import json
import datetime
from asyncua import Server, ua

# --- [1. 설정 정보] ---
COM_PORT = 'COM4'
MY_IP = '0.0.0.0'
YOLO_PORT = 6000
OPC_UA_PORT = 40000

# 서버 내부 상태 관리
state = {
    "target_state": 1, 
    "t_spd_main": 50, "t_spd_sort": 50, "t_spd_load": 50,
    "t_flags": 0, "scan_result": 0
}

# 상태 락 (동시 접근 방지)
state_lock = asyncio.Lock()

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

# --- [3. MCU 수신 보고 핸들러 (동기화 개선)] ---
async def handle_mcu_report(ser, r):
    buffer = bytearray()
    
    while True:
        try:
            # 사용 가능한 데이터 읽기
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
            
            # 헤더 찾기
            while len(buffer) >= 8:
                # 헤더(0xFE) 찾기
                if buffer[0] != 0xFE:
                    # 잘못된 데이터 버리기
                    try:
                        header_idx = buffer.index(0xFE)
                        buffer = buffer[header_idx:]
                    except ValueError:
                        buffer.clear()
                        break
                
                # 완전한 패킷인지 확인
                if len(buffer) >= 8:
                    if buffer[7] == 0xFF:
                        # 유효한 패킷 처리
                        raw = buffer[:8]
                        buffer = buffer[8:]
                        
                        await r['state'].write_value(ua.Variant(raw[1], ua.VariantType.Int64))
                        await r['s1'].write_value(ua.Variant(raw[2], ua.VariantType.Int64))
                        await r['s2'].write_value(ua.Variant(raw[3], ua.VariantType.Int64))
                        await r['s3'].write_value(ua.Variant(raw[4], ua.VariantType.Int64))
                        await r['floor'].write_value(ua.Variant(raw[6], ua.VariantType.Int64))
                        
                        dev = raw[5]
                        await r['is_busy'].write_value(bool((dev >> 2) & 0x01))
                        await r['is_robot'].write_value(bool((dev >> 5) & 0x01))
                        await r['robot_done'].write_value(bool((dev >> 6) & 0x01))
                    else:
                        # 테일러가 잘못됨 - 1바이트 버리고 다시 찾기
                        buffer = buffer[1:]
                else:
                    break
                    
        except Exception as e:
            print(f"❌ MCU 수신 에러: {e}")
        
        await asyncio.sleep(0.01)

# --- [4. YOLO 클라이언트 개별 처리] ---
async def handle_yolo_client(client, addr, ser, box_node):
    """개별 YOLO 클라이언트 처리 (비동기)"""
    try:
        loop = asyncio.get_running_loop()
        data = await loop.sock_recv(client, 1024)
        
        if data:
            raw_data = data.decode('utf-8', errors='ignore').strip().upper()
            val = 0
            
            # 정확한 매칭으로 변경
            if raw_data == "LARGE":
                val = 1
            elif raw_data == "SMALL":
                val = 2
            
            if val > 0:
                async with state_lock:
                    state["scan_result"] = val
                    print(f"📸 [YOLO -> PC] 감지 결과: {val} ({raw_data})")
                    
                    # MCU에 즉시 전송
                    send_to_stm32(ser)
                
                # OPC UA 노드 업데이트
                json_string = json.dumps({
                    "size": "LARGE" if val == 1 else "SMALL",
                    "timestamp": datetime.datetime.utcnow().isoformat()
                })
                await box_node.write_value(ua.Variant(json_string, ua.VariantType.String))
                
                # 전송 후 일정 시간 후 초기화 (MCU 처리 시간 확보)
                await asyncio.sleep(0.5)
                async with state_lock:
                    state["scan_result"] = 0
                    
    except Exception as e:
        print(f"❌ YOLO 클라이언트 처리 에러 ({addr}): {e}")
    finally:
        client.close()

# --- [5. YOLO 수신 핸들러 (동시 연결 처리)] ---
async def handle_yolo(ser, box_node):
    loop = asyncio.get_running_loop()
    server_soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_soc.bind((MY_IP, YOLO_PORT))
    server_soc.listen(5)
    server_soc.setblocking(False)
    print(f"✅ YOLO 서버 대기 중 (Port: {YOLO_PORT})")

    while True:
        try:
            client, addr = await loop.sock_accept(server_soc)
            print(f"🔗 YOLO 클라이언트 연결: {addr}")
            # 각 연결을 별도 태스크로 처리
            asyncio.create_task(handle_yolo_client(client, addr, ser, box_node))
        except Exception as e:
            print(f"❌ YOLO 연결 수락 에러: {e}")
            await asyncio.sleep(0.1)

# --- [6. 메인 서버 및 노드 생성] ---
async def main():
    try:
        ser = serial.Serial(COM_PORT, 115200, timeout=0.1)
        print(f"✅ MCU 연결 성공: {COM_PORT}")
    except Exception as e:
        print(f"❌ {COM_PORT} 연결 실패: {e}")
        return

    ua_server = Server()
    await ua_server.init()
    ua_server.set_endpoint(f"opc.tcp://{MY_IP}:{OPC_UA_PORT}")
    ua_server.set_server_name("STM32_Gateway")
    ua_server.set_security_policy([ua.SecurityPolicyType.NoSecurity])

    idx = await ua_server.register_namespace("SMART_FACTORY")
    objects = ua_server.nodes.objects

    # 명령 노드
    n_t_state = await objects.add_variable(ua.NodeId(40001, idx), "TargetState", ua.Variant(1, ua.VariantType.Int64))
    n_t_s1 = await objects.add_variable(ua.NodeId(40002, idx), "TargetSpeedMain", ua.Variant(50, ua.VariantType.Int64))
    n_t_s2 = await objects.add_variable(ua.NodeId(40003, idx), "TargetSpeedSort", ua.Variant(50, ua.VariantType.Int64))
    n_t_s3 = await objects.add_variable(ua.NodeId(40004, idx), "TargetSpeedLoad", ua.Variant(50, ua.VariantType.Int64))
    n_agv_s_arr = await objects.add_variable(ua.NodeId(40005, idx), "AgvSortArrived", False)
    n_agv_s_dep = await objects.add_variable(ua.NodeId(40006, idx), "AgvSortDeparted", False)
    n_agv_l_arr = await objects.add_variable(ua.NodeId(40007, idx), "AgvLoadArrived", False)
    n_agv_l_dep = await objects.add_variable(ua.NodeId(40008, idx), "AgvLoadDeparted", False)

    for n in [n_t_state, n_t_s1, n_t_s2, n_t_s3, n_agv_s_arr, n_agv_s_dep, n_agv_l_arr, n_agv_l_dep]:
        await n.set_writable()

    # 보고 노드
    r_nodes = {
        'state': await objects.add_variable(ua.NodeId(50001, idx), "CurrentState", ua.Variant(0, ua.VariantType.Int64)),
        's1':    await objects.add_variable(ua.NodeId(50002, idx), "CurrentSpeedMain", ua.Variant(0, ua.VariantType.Int64)),
        's2':    await objects.add_variable(ua.NodeId(50003, idx), "CurrentSpeedSort", ua.Variant(0, ua.VariantType.Int64)),
        's3':    await objects.add_variable(ua.NodeId(50004, idx), "CurrentSpeedLoad", ua.Variant(0, ua.VariantType.Int64)),
        'floor': await objects.add_variable(ua.NodeId(50005, idx), "CurrentFloor", ua.Variant(0, ua.VariantType.Int64)),
        'is_busy':    await objects.add_variable(ua.NodeId(50006, idx), "IsLiftMoving", False),
        'is_robot':   await objects.add_variable(ua.NodeId(50007, idx), "IsRobotWorking", False),
        'robot_done': await objects.add_variable(ua.NodeId(50008, idx), "IsRobotDone", False),
        'box_detect': await objects.add_variable(ua.NodeId(50009, idx), "BoxCreated", "")
    }

    asyncio.create_task(handle_mcu_report(ser, r_nodes))
    asyncio.create_task(handle_yolo(ser, r_nodes['box_detect']))

    async with ua_server:
        print(f"🚀 OPC UA 서버 가동 중 (Port: {OPC_UA_PORT})")
        while True:
            try:
                v_st = await n_t_state.read_value()
                v_s1, v_s2, v_s3 = await n_t_s1.read_value(), await n_t_s2.read_value(), await n_t_s3.read_value()
                f0, f1 = await n_agv_s_arr.read_value(), await n_agv_s_dep.read_value()
                f2, f3 = await n_agv_l_arr.read_value(), await n_agv_l_dep.read_value()
                new_flags = (int(f0) << 0) | (int(f1) << 1) | (int(f2) << 2) | (int(f3) << 3)

                async with state_lock:
                    if (v_st != state["target_state"] or new_flags != state["t_flags"] or 
                        v_s1 != state["t_spd_main"] or v_s2 != state["t_spd_sort"] or v_s3 != state["t_spd_load"]):
                        state.update({"target_state": v_st, "t_flags": new_flags, "t_spd_main": v_s1, "t_spd_sort": v_s2, "t_spd_load": v_s3})
                        send_to_stm32(ser)

            except Exception as e:
                print(f"❌ 메인 루프 에러: {e}")
            
            await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())