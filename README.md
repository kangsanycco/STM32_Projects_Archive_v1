
[ 폴더 종류 ]

1. Practice
 - 미디어 및 서적과 AI를 활용하여 연습한 프로젝트 목록입니다. 팀 프로젝트에 적응하기 위한 발판이 되었습니다.
 - STM32CubeIDE를 활용하여 연습했습니다.
 - GPIO, EXTI, UART, DMA, ADC, TIMER, PWM, FreeRTOS 등의 연습 내용이 담겨있습니다.

2. Team_Project 
 - R.O.S 팀의 임베디드 기술 코드가 담겨있는 코드입니다.
 - STM32CubeIDE를 활용하여 구현했습니다.
 - 단순 main.c에 머무르지 않고, 추상화 계층을 사용하여 Core 내 파일을 세분화하였습니다.

3. MyServer
 - R.O.S 팀의 STM32, Yolo, OPC-UA 의 중계 서버를 구현한 코드가 담겨있습니다.
 - 파이썬을 활용하여 서버를 구현했습니다.
 - 이기종간 데이터의 규격을 맞추고 송수신 로직이 포함되어 있습니다.




[ Team_Project ]

 - 이 프로젝트는 추상화 계층형 구조로 설계되었습니다. 불필요한 정보 노출을 막고 코드의 독립성을 유지할 수 있습니다.
 - 크게 Common, Drv, Bsp, Task, App 구조로 나누었습니다.

1. Common (Library & Protocol) :  모든 계층이 공유하는 전역 장부와 규칙. 외부 계층의 헤더를 전부 가져옴.
 - config.h : 하드웨어 핀 번호와 리니어 층별 펄스 값 등 모든 물리적 상수 정의.
 - system_state.h : 공정 단계(FSM), 센서 실시간 값, 모터 현재 위치를 담은 공유 장부.
 - function.h : 모든 계층이 공통으로 사용하는 표준 라이브러리 및 함수 선언 집합.


2. Drv (Driver Layer) : MCU 핀을 통한 전기 신호 입출력
 - drv_exti.c : 모든 센서와 완료 신호의 인터럽트를 감지하여 즉시 장부에 기록하는 입구
 - drv_uart.c : DMA, EXTI를 이용해 PC와 데이터를 주고받는 통신 통로.
 - drv_i2c.c : PCA9685 드라이버와 통신하여 서보 모터 데이터를 보내는 통신 통로.


3. Bsp (Board Support Package Layer) : 전기 신호를 장치 단위(모터, 센서)로 통역.
 - bsp_sensor.c : 물리적인 High/Low 신호를 "물체 있음/없음"이라는 의미 있는 데이터로 변환.
 - bsp_servo.c : I2C 통신 데이터를 활용해 컨베이어와 분류 서보를 원하는 속도로 제어.
 - bsp_stepper.c : TB6600 드라이버에 맞춰 리니어 모터의 펄스(Pulse)와 방향(Dir)을 제어.


4. Task (Scheduling Layer) : 정해진 주기마다 일을 시키는 실행 관리. 언제 실행되는 지 판단 
 - task_system.c : 모든 신호 상태 최신화, 공정 제어 갱신


5. App (Application Layer) : 공정의 시나리오를 결정하는 최종 판단 두뇌.
 - app_fsm.c : 장부를 실시간으로 읽어 다음 공정 단계를 결정하는 두뇌.
 - app_fsm.h: app_fsm.c 의 헤더


6. Src
 - main.c : 초기화 함수를 관리하고, FreeRTOS 커널을 시작시키는 최상위 실행점




