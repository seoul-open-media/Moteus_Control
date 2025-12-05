# Branch: electromagnet-solenoid-control

## 개요
XBee 무선 통신을 통한 다중 로봇 제어 시스템. Pure Data와 통합하여 13개 로봇을 동시에 제어하며, 전자석/솔레노이드 액추에이터 제어 기능 추가.

## 하드웨어
- **마이크로컨트롤러**: Teensy MicroMod
- **모터 컨트롤러**: 2x Moteus R4.11 (ID 1, 2)
- **펌웨어**: Moteus v268 (안정적 작동 확인)
- **라이브러리**: mjbots/Moteus@^1.0.2
- **무선 통신**: XBee (Serial4, 115200 baud)
- **디스플레이**: OLED SSD1306 128x64 (Wire1, I2C)
- **버튼**: Adafruit NeoKey 1x4 (Wire, I2C)
- **액추에이터**: 전자석 3개 (EM1, EM2, EM3), 솔레노이드 1개

## 주요 기능

### 1. XBee 무선 통신 프로토콜
**메시지 구조** (28 바이트):
- Byte 1: `0xFF` (헤더)
- Byte 2: 제어 바이트
  - `0` = 정상 작동
  - `1` = 긴급 정지 (모든 모터 정지, XBee 제어 비활성화)
- Bytes 3-28: 13개 로봇 데이터 (각 2바이트 MSB/LSB)

**브로드캐스트 모드**:
- 모든 26바이트가 `0xFF`이면 전체 로봇이 동일한 명령 실행

**로봇별 제어**:
- Robot ID (1-13)에 따라 해당하는 2바이트 읽기
- 16비트 값을 4자리 숫자로 파싱 (예: 5127)

### 2. 4자리 제어 코드
각 로봇은 4자리 숫자로 제어됨 (예: `5127`):

**digit1 (천의 자리)**: 속도 (0-9)
```cpp
0 → 0 rev/s
1 → 10 rev/s
2 → 20 rev/s
3 → 30 rev/s
4 → 40 rev/s
5 → 50 rev/s
6 → 60 rev/s
7 → 70 rev/s
8 → 80 rev/s
9 → 100 rev/s
```

**digit2 (백의 자리)**: 모터 1 위치
```cpp
0 → 현재 목표 유지
1 → -0.25 회전 (-90°)
2 → -0.167 회전 (-60°)
3 → -0.083 회전 (-30°)
4 → 0.0 회전 (0°)
5 → 0.0 회전 (0°, 중앙)
6 → 0.083 회전 (30°)
7 → 0.167 회전 (60°)
8 → 0.25 회전 (90°)
9 → 0.25 회전 (90°)
```

**digit3 (십의 자리)**: 모터 2 위치 (digit2와 동일한 매핑)

**digit4 (일의 자리)**: 전자석/솔레노이드 제어

### 3. 로봇 타입별 액추에이터 제어

**Type A** (`ROBOT_TYPE = 'A'` in config.h):
- digit4 = `1`: EM1, EM2, EM3 활성화
- digit4 = `2`: 솔레노이드 활성화
- digit4 = `3`: 모두 활성화

**Type B** (`ROBOT_TYPE = 'B'` in config.h):
- digit4 = `1`: EM1, EM2 활성화
- digit4 = `2`: 솔레노이드, EM3 활성화
- digit4 = `3`: 모두 활성화

### 4. 속도 제어 모드
**Position = NaN 방식**:
- 위치 제어를 비활성화하고 속도만 제어
- 소프트웨어 PD 컨트롤러로 목표 속도 계산
- 부드러운 가속/감속 구현

**PD 제어 알고리즘**:
```cpp
// 목표 위치와의 오차 계산
float error = target - current_position;

// 랩어라운드 처리
if (error > 0.5) error -= 1.0;
else if (error < -0.5) error += 1.0;

// 브레이크 존 판단
const float brake_zone = 0.01;  // 3.6도

if (abs(error) > brake_zone) {
  // 멀리 있을 때: 빠른 접근
  velocity = error * Kp_far - current_velocity * Kd;
} else {
  // 가까이 있을 때: 부드러운 접근
  velocity = error * Kp_near - current_velocity * (Kd * 0.3);
}
```

**제어 파라미터**:
- `maximum_torque = 1.2` Nm (저속 토크 향상)
- `brake_zone = 0.01` 회전 (~3.6도)
- `damping_factor = 0.3` (목표 근처에서)
- `deadband = 0.003` (저속), `0.01` (고속)
- `velocity_deadband = 0` (저속), `3.0` rev/s (고속)
- `accel_limit = 800.0`

### 5. 디스플레이 시스템
**OLED 레이아웃**:
```
   Motor1  Motor2
pos.  45°     -30°
targ.  0°      0°
temp. 35C     37C

Status: ENGAGED
```

**특징**:
- 위치를 각도로 표시 (-90° ~ +90°)
- 목표 위치 실시간 표시
- XBee 제어 상태 표시 (ENGAGED/DISENGAGED)
- 온도 모니터링

**위치 변환**:
```cpp
// 회전수를 -0.25~0.25 범위로 정규화
m1_pos = fmod(m1_pos + 0.25, 0.5) - 0.25;
// 각도로 변환
float m1_deg = m1_pos * 360.0;  // -90° ~ +90°
```

### 6. NeoKey 버튼 제어
**Key 1 (초록색)**: 기본 위치
- 두 모터를 0° 위치로 이동
- 속도 1 (10 rev/s) 사용
- XBee 제어 활성화

**Key 2 (파란색)**: 위치 0.25
- 두 모터를 0.25 회전 위치로 이동

**Key 3 (청록색)**: 위치 0.75
- 두 모터를 0.75 회전 위치로 이동

**Key 4 (빨간색)**: 긴급 정지
- 즉시 모든 모터 정지
- XBee 제어 비활성화

### 7. 성능 최적화

**진동/떨림 제거**:
- 댐핑 계수를 단계적으로 감소 (1.5 → 0.8 → 0.3)
- 속도 의존적 데드밴드 구현
- 브레이크 존 최적화

**위치 정확도 개선**:
- 저속 시 데드밴드 0.003 (±1도)
- 목표 위치 근처에서 속도 데드밴드 제거
- 여러 번 측정하여 도달 확인

**접근 속도 개선**:
- 브레이크 존 축소 (0.03 → 0.01)
- 댐핑 최소화로 속도 유지
- PID 게인 동적 조정

**Moteus PID 설정**:
```bash
moteus_tool --target 1 -c
conf set servo.pid_position.kd 0.5
conf write
```

### 8. 전자석/솔레노이드 제어
**타이밍 파라미터** (solenoid.cpp):
```cpp
#define EM_DURATION_MS 100      // 전자석 활성 시간
#define SOL_DURATION_MS 50      // 솔레노이드 활성 시간
```

**트리거 함수**:
- `triggerTypeA_1()`: EM1, EM2, EM3
- `triggerTypeA_2()`: 솔레노이드
- `triggerTypeA_3()`: 모두
- `triggerTypeB_1()`: EM1, EM2
- `triggerTypeB_2()`: 솔레노이드, EM3
- `triggerTypeB_3()`: 모두

**비차단 타이머**:
- `updateSolenoid()` 함수가 메인 루프에서 호출됨
- 타이머 만료 시 자동으로 비활성화
- 여러 액추에이터 독립적으로 제어

## 파일 구조

### 핵심 파일
```
src/
├── main.cpp              메인 루프, 초기화
├── config.h              로봇 ID, 타입, 핀 설정
├── xbee.cpp/h            XBee 통신, 제어 알고리즘
├── motor_control.cpp/h   모터 명령 함수
├── display.cpp/h         OLED 디스플레이
├── neokey.cpp/h          NeoKey 버튼 처리
└── solenoid.cpp/h        전자석/솔레노이드 제어
```

### config.h 설정
```cpp
#define ROBOT_ID 1           // 로봇 번호 (1-13)
#define ROBOT_TYPE 'A'       // 'A' 또는 'B'
#define XBEE_BAUD 115200     // XBee 통신 속도

// 위치 제약 (-90° ~ +90°)
const float MIN_POSITION = -0.25;
const float MAX_POSITION = 0.25;
```

## 주요 함수

### XBee 통신
**`updateXBee()`**:
- XBee Serial4에서 데이터 수신
- 헤더 바이트 (0xFF 0xFF) 탐색
- 제어 바이트 확인
- 로봇별 데이터 파싱
- 목표 위치/속도 업데이트

**`updateXBeeControl()`**:
- 100Hz (10ms)로 실행
- 현재 위치/속도 쿼리
- 오차 계산 및 PD 제어
- 모터 명령 전송
- 정착 상태 확인

### 모터 제어
**`moveToEncoderPosition()`**:
- 목표 위치로 이동
- 블로킹 방식 (완료까지 대기)
- 타임아웃 15초

## 디버깅

### 시리얼 모니터 출력
```
[XBee] Header found! Control byte: 0
[XBee] Normal operation mode
Robot 1 MSB: 50 LSB: 127
Parsed for Robot 1 value: 5127 digits: v=5 m1=1 m2=2 d4=7
[Mapping] vel=50.0 d2=1 -> pos1=-0.2500 d3=2 -> pos2=-0.1667
[XBee] Motor targets: M1=-0.250 M2=-0.167 vel=50.0

[Query] M1 pos=-0.245 temp=38.5 M2 pos=-0.162 temp=39.2
[Err] M1: err=-0.005 M2: err=-0.005
[Vel] max_vel_m1=50.0 max_vel_m2=50.0
[Calc] vel1=-0.50 (lim=50) vel2=-0.83 (lim=50)
M1 settled
M2 settled

[NeoKey1] Set targets: M1=0.0000 M2=0.0000 vel=10.0 active=YES
[XBee Control] Active, targets: M1=0.0000 M2=0.0000
[Display] Targets: M1=0.0000 (0deg) M2=0.0000 (0deg)
```

### 문제 해결

**타겟이 표시되지 않음 (-- --):**
- 제어 바이트가 1 (정지)인지 확인
- Pure Data 패치에서 제어 바이트 0 전송
- XBee 배선 확인

**모터가 움직이지 않음:**
- `xbeeControlActive` 상태 확인
- 시리얼 모니터에서 `[XBee Control] Active` 메시지 확인
- 목표 위치가 현재 위치와 다른지 확인

**진동/떨림:**
- 데드밴드 증가 (현재 0.003)
- 댐핑 증가 (현재 0.3)
- 브레이크 존 증가 (현재 0.01)

## Pure Data 통합

### 메시지 생성 예제
```
# Pure Data 패치에서:
# 로봇 1번에게 속도 5, 위치 M1=1, M2=2, digit4=7 전송

1. 4자리 숫자 생성: 5127
2. MSB/LSB 변환: MSB=50 (5127/256), LSB=127 (5127%256)
3. 메시지 조립:
   - Byte 1: 255 (0xFF)
   - Byte 2: 0 (정상 작동)
   - Bytes 3-4: 50 127 (로봇 1)
   - Bytes 5-28: 각 로봇 데이터 또는 0
4. XBee로 전송
```

### 브로드캐스트 예제
```
# 모든 로봇을 속도 3, 중앙 위치로 이동
1. 4자리 숫자: 3550 (속도 3, 위치 5/5, digit4=0)
2. MSB/LSB: MSB=13, LSB=206
3. 모든 26바이트를 255로 채움
4. 전송
```

## 성능 특성

✅ **안정성**:
- Moteus v268 펌웨어 검증 완료
- Teensy MicroMod 사용
- 100Hz 제어 루프

✅ **성능**:
- 저속 토크 1.2 Nm
- 위치 정확도 ±1도
- 응답 시간 < 100ms
- 13개 로봇 동시 제어

✅ **사용성**:
- 각도 단위 디스플레이
- NeoKey 단축키
- Pure Data 통합
- 실시간 상태 모니터링

## 업데이트 기록

### v1.0 (초기 버전)
- XBee 프로토콜 구현
- 기본 위치 제어

### v1.1 (성능 개선)
- 속도 제어 모드로 전환
- 토크 증가 (1.2 Nm)
- 진동 제거

### v1.2 (디스플레이 개선)
- 각도 표시 (-90° ~ +90°)
- 목표 위치 표시
- 속도 매핑 조정

### v1.3 (NeoKey 통합)
- NeoKey 버튼 기능 추가
- 긴급 정지 구현
- XBee 제어 변수 통합

### v1.4 (전자석/솔레노이드)
- Type A/B 구현
- digit4 제어 추가
- 비차단 타이머 구현

### v1.5 (최적화)
- 브레이크 존 축소 (0.01)
- 댐핑 최소화 (0.3)
- 데드밴드 동적 조정
- 디버그 출력 추가
