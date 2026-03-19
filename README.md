# [HW/FW] R.O.S Team Project (~2026.02.10)

코드랩 아카데미 팀 프로젝트 (9인, 2026.01 ~ 2026.02)

---

## 프로젝트 개요
산업용 컨베이어 기반 자동 물품 분류·이송·로봇 작업 시스템 구현.  
STM32 중심의 실시간 제어(FreeRTOS, FSM, UART DMA, I2C PWM)를 담당하였으며, PL 역할로 기획·일정 관리·팀 조율 수행

---

## 폴더 구조

</details>

<details open>
<summary>📂 Team_Project - 본 프로젝트 핵심 코드 (R.O.S 팀 임베디드 부분)</summary>

STM32CubeIDE 기반으로 구현한 임베디드 FW 코드입니다.  
단순 main.c에 모든 로직을 넣지 않고, **추상화 계층 구조**를 도입하여 유지보수성과 확장성을 높였습니다.

### 계층 구조
- **Common** (공유 라이브러리 & 프로토콜)  
  모든 계층이 공유하는 전역 상수·구조체·헤더  
  - `config.h`: 핀 번호, 리니어 펄스 값 등 물리적 상수 정의  
  - `system_state.h`: FSM 상태, 센서 실시간 값, 모터 위치 등 공유 장부  
  - `function.h`: 공통 함수 선언 집합

- **Drv** (Driver Layer) - MCU 핀을 통한 전기 신호 입출력  
  - `drv_exti.c`: 센서·완료 신호 인터럽트 처리  
  - `drv_uart.c`: DMA + EXTI 기반 PC/AGV 통신  
  - `drv_i2c.c`: PCA9685 모터 드라이버 통신

- **Bsp** (Board Support Package) - 신호를 장치 단위로 변환  
  - `bsp_sensor.c`: High/Low 신호 → "물체 있음/없음" 의미 부여  
  - `bsp_servo.c`: I2C 데이터 → 컨베이어 속도 제어  
  - `bsp_robot.c`: 로봇 작동 신호 송신

- **Task** (Scheduling Layer) - 주기적 실행 관리  
  - `task_system.c`: 신호 상태 업데이트, 공정 갱신

- **App** (Application Layer) - 최종 판단 로직  
  - `app_fsm.c`: 실시간 장부 읽어 다음 공정 단계 결정 (FSM 구현)

- **Src**  
  - `main.c`: 초기화 + FreeRTOS 커널 시작

</details>

<details>
<summary>📂 MyServer - 중계 서버 (팀원 담당, 참고용)</summary>

STM32 ↔ YOLO ↔ OPC-UA 중계 서버 (Python 구현)  
이기종 데이터 규격 맞춤 및 송수신 로직 포함.

</details>

---

## 기술 스택
- **FW**: C (STM32CubeIDE), FreeRTOS (멀티태스킹·우선순위 제어), FSM 상태 머신  
- **통신/제어**: UART DMA, I2C (PCA9685 PWM), PWM/EXTI  
- **디버깅**: Breakpoint, Live Expression  
- **HW 검증**: 멀티미터, 데이터시트 분석, 전원 안정화 (커패시터·저항 배치)

---


