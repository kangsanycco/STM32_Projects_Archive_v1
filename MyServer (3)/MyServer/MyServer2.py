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

# --- [2. MCU 송신 로직] ---
def send_to_stm32(ser):
    """
    packet[6] = scan_result (1: LARGE, 0: 나머지/대기)
    """
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
        try:
            if ser.in_waiting >= 8:
                raw = ser.read(8)
                if len(raw) == 8 and raw[0] == 0xFE and raw[7] == 0xFF:
                    await r['state'].write_value(ua.Variant(int(raw[1]), ua.VariantType.Int64))
                    await r['s1'].write_value(ua.Variant(int(raw[2]), ua.VariantType.Int64))
                    await r['s2'].write_value(ua.Variant(int(raw[3]), ua.VariantType.Int64))
                    await r['s3'].write_value(ua.Variant(int(raw[4]), ua.VariantType.Int64))
                    await r['floor'].write_value(ua.Variant(int(raw[6]), ua.VariantType.Int64))

                    dev = raw[5]
                    await r['is_busy'].write_value(bool((dev >> 2) & 0x01))
                    await r['is_robot'].write_value(bool((dev >> 5) & 0x01))
                    await r['robot_done'].write_value(bool((dev >> 6) & 0x01))
        except Exception as e:
            print(f"❌ MCU 수신/OPC 업데이트 에러: {e}")

        await asyncio.sleep(0.01)

# --- [4. YOLO 수신 핸들러] ---
async def handle_yolo(ser, box_node):
    """
    ✅ 수신 포맷 2개 모두 지원
    1) "PASS seq=.. size=LARGE ..." / "size=SMALL"
    2) "box,2" / "box,1"   (여기서 2=LARGE 라고 사용자 확인)
    """
    loop = asyncio.get_running_loop()

    server_soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_soc.bind((MY_IP, YOLO_PORT))
    server_soc.listen(5)
    server_soc.setblocking(False)

    print(f"✅ YOLO 서버 대기 중 (Port: {YOLO_PORT})")

    while True:
        client, addr = await loop.sock_accept(server_soc)
        client.setblocking(False)  # ✅ 중요
        print(f"🔌 YOLO client connected: {addr}")

        buf = b""
        try:
            while True:
                chunk = await loop.sock_recv(client, 1024)
                if not chunk:
                    print("🔌 YOLO client disconnected")
                    break

                buf += chunk

                # 라인(\n) 단위 처리 (우분투가 \n 없이 보낼 수도 있어서, 그 경우는 아래 fallback도 있음)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    raw_data = line.decode(errors="ignore").strip()
                    if not raw_data:
                        continue

                    # --- 여기서부터 "판별" ---
                    print("RAW:", raw_data)

                    is_large = False
                    is_small = False

                    # (A) 문자열 포맷: size=LARGE / size=SMALL
                    if "size=LARGE" in raw_data:
                        is_large = True
                    elif "size=SMALL" in raw_data:
                        is_small = True

                    # (B) CSV 포맷: box,2 / box,1 (2=LARGE 확정)
                    if (not is_large and not is_small) and ("," in raw_data):
                        parts = [p.strip() for p in raw_data.split(",") if p.strip() != ""]
                        # 마지막 토큰을 숫자로 해석
                        try:
                            size_num = int(parts[-1])
                            if size_num == 2:
                                is_large = True
                            elif size_num == 1 or size_num == 0:
                                is_small = True
                        except:
                            pass

                    if not (is_large or is_small):
                        # 이해 못한 메시지면 무시
                        continue

                    # 1) STM32 전송용 값 결정 (LARGE만 1)
                    state["scan_result"] = 1 if is_large else 0

                    # 2) MCU 즉시 전송
                    send_to_stm32(ser)
                    print(f"📸 [YOLO -> PC] {'LARGE' if is_large else 'SMALL'} 감지 처리 완료")

                    # 3) OPC UA 업데이트 (원문 + 해석 결과 기록)
                    json_string = json.dumps({
                        "size": "LARGE" if is_large else "SMALL",
                        "raw": raw_data,
                        "timestamp": datetime.datetime.utcnow().isoformat()
                    })
                    await box_node.write_value(ua.Variant(json_string, ua.VariantType.String))

                    # 4) 엣지 트리거 리셋
                    state["scan_result"] = 0

                # ✅ 만약 우분투가 '\n' 없이 짧게 한번만 보내는 경우 대비(fallback)
                # 버퍼가 너무 커지면 그냥 한 덩어리로 처리해버림
                if len(buf) > 2048 and b"\n" not in buf:
                    raw_data = buf.decode(errors="ignore").strip()
                    buf = b""
                    if raw_data:
                        print("RAW(FALLBACK):", raw_data)
                        # 위와 동일한 판별 로직 재사용
                        is_large = ("size=LARGE" in raw_data)
                        is_small = ("size=SMALL" in raw_data)
                        if not (is_large or is_small) and ("," in raw_data):
                            try:
                                size_num = int(raw_data.split(",")[-1].strip())
                                if size_num == 2:
                                    is_large = True; is_small = False
                                elif size_num == 1 or size_num == 0:
                                    is_small = True; is_large = False
                            except:
                                pass

                        if is_large or is_small:
                            state["scan_result"] = 1 if is_large else 0
                            send_to_stm32(ser)
                            json_string = json.dumps({
                                "size": "LARGE" if is_large else "SMALL",
                                "raw": raw_data,
                                "timestamp": datetime.datetime.utcnow().isoformat()
                            })
                            await box_node.write_value(ua.Variant(json_string, ua.VariantType.String))
                            state["scan_result"] = 0

        except Exception as e:
            print(f"❌ YOLO 수신 루프 에러: {e}")
        finally:
            try:
                client.close()
            except:
                pass

# --- [5. 메인 서버 및 노드 생성] ---
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
        'box_detect': await objects.add_variable(ua.NodeId(50009, idx), "BoxCreated", ua.Variant("", ua.VariantType.String))
    }

    asyncio.create_task(handle_mcu_report(ser, r_nodes))
    asyncio.create_task(handle_yolo(ser, r_nodes['box_detect']))

    async with ua_server:
        print(f"🚀 OPC UA 서버 가동 중 (ns={idx}, Port: {OPC_UA_PORT})")
        while True:
            v_st = await n_t_state.read_value()
            v_s1, v_s2, v_s3 = await n_t_s1.read_value(), await n_t_s2.read_value(), await n_t_s3.read_value()
            f0, f1, f2, f3 = await n_agv_s_arr.read_value(), await n_agv_s_dep.read_value(), await n_agv_l_arr.read_value(), await n_agv_l_dep.read_value()
            new_flags = (int(f0) << 0) | (int(f1) << 1) | (int(f2) << 2) | (int(f3) << 3)

            if (v_st != state["target_state"] or new_flags != state["t_flags"] or
                v_s1 != state["t_spd_main"] or v_s2 != state["t_spd_sort"] or v_s3 != state["t_spd_load"]):
                state.update({"target_state": int(v_st), "t_flags": int(new_flags),
                              "t_spd_main": int(v_s1), "t_spd_sort": int(v_s2), "t_spd_load": int(v_s3)})
                send_to_stm32(ser)

            await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())