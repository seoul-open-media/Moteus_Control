# Moteus Control - 브랜치 요약

## 프로젝트 개요
Moteus R4.11 모터 컨트롤러를 사용하는 로봇 제어 시스템. 각 브랜치는 서로 다른 제어 방식과 하드웨어 구성을 가지고 있습니다.

---

## main (기본 브랜치)

### 핵심 특징
- **제어 방식**: 외부 AS5600 엔코더 기반 위치 제어
- **하드웨어**: CANBed FD, Teensy (teensymm board)
- **엔코더**: AS5600 자기 엔코더 (AUX2 포트 연결, 21:1 기어비)
- **인터페이스**: 시리얼 명령어, NeoKey 1x4 버튼

### 제어 알고리즘
- **속도 제어 기반**: position = NaN, 속도를 직접 제어
- **3단계 제어**:
  1. 정상 이동 (15 rev/s)
  2. 감속 구간 (0.10 rev 이내)
  3. 브레이킹 (0.06 rev 이내, 2.0 rev/s)
- **랩어라운드 처리**: 0/1 경계를 최단 경로로 이동
- **오버슛 감지**: 목표 통과 시 자동 브레이킹

### 주요 파라미터
```cpp
TOLERANCE = 0.05      // 위치 정확도
MAX_VELOCITY = 15.0   // 최대 속도
SLOW_ZONE = 0.10      // 감속 시작 거리
BRAKE_ZONE = 0.06     // 브레이킹 시작 거리
LOOP_PERIOD_MS = 10   // 제어 주기 (100Hz)
```

### 사용 사례
- 정밀한 위치 제어가 필요한 경우
- 외부 엔코더 사용 (기어 감속비 적용)
- 랩어라운드가 있는 회전 시스템
- 단일 로봇 또는 소수 로봇 제어

### 파일 구조
```
src/
├── main.cpp
├── config.h
├── motor_control.cpp/h
├── display.cpp/h
└── neokey.cpp/h
```

---

## electromagnet-solenoid-control (현재 브랜치)

### 핵심 특징
- **제어 방식**: XBee 무선 통신 기반 다중 로봇 제어
- **하드웨어**: Teensy MicroMod, Moteus v268 펌웨어
- **통신**: XBee Serial4 (115200 baud)
- **액추에이터**: 전자석 3개 + 솔레노이드 1개
- **인터페이스**: Pure Data 통합, NeoKey 버튼

### 제어 알고리즘
- **속도 제어 + 소프트웨어 PD**: position = NaN
- **2단계 제어**:
  1. 고속 접근 (brake_zone 외부)
  2. 저속 접근 (brake_zone 내부, 댐핑 0.3)
- **동적 데드밴드**: 속도에 따라 0.003 ~ 0.01
- **속도 의존 PID 게인**: 선형 스케일링

### 주요 파라미터
```cpp
maximum_torque = 1.2       // 저속 토크 향상
brake_zone = 0.01          // 3.6도
damping_factor = 0.3       // 목표 근처
deadband = 0.003/0.01      // 저속/고속
velocity_deadband = 0/3.0  // 저속/고속
accel_limit = 800.0
```

### XBee 프로토콜
**메시지 구조** (28 바이트):
```
Byte 1: 0xFF (헤더)
Byte 2: 제어 바이트 (0=정상, 1=정지)
Bytes 3-28: 13개 로봇 데이터 (각 2바이트)
```

**4자리 제어 코드** (예: 5127):
- digit1: 속도 (0-9 → 0-100 rev/s)
- digit2: 모터1 위치 (0-9 → -90° ~ +90°)
- digit3: 모터2 위치 (0-9 → -90° ~ +90°)
- digit4: 전자석/솔레노이드 제어

### 로봇 타입
**Type A**:
- digit4=1: EM1, EM2, EM3
- digit4=2: 솔레노이드
- digit4=3: 모두

**Type B**:
- digit4=1: EM1, EM2
- digit4=2: 솔레노이드, EM3
- digit4=3: 모두

### 사용 사례
- 다중 로봇 동시 제어 (최대 13대)
- Pure Data와 실시간 통합
- 무선 제어 필요 시
- 전자석/솔레노이드 액추에이터 제어
- 예술 설치 작품, 공연

### 파일 구조
```
src/
├── main.cpp
├── config.h
├── xbee.cpp/h              ← 추가
├── motor_control.cpp/h
├── display.cpp/h
├── neokey.cpp/h
└── solenoid.cpp/h          ← 추가
```

---

## 브랜치 비교표

| 특성 | main | electromagnet-solenoid-control |
|------|------|-------------------------------|
| **제어 방식** | 외부 엔코더 | XBee 무선 통신 |
| **마이크로컨트롤러** | Teensy (teensymm) | Teensy MicroMod |
| **CAN 인터페이스** | CANBed FD | MCP2517FD (내장) |
| **Moteus 펌웨어** | 호환 버전 | v268 |
| **엔코더** | AS5600 (AUX2) | Moteus 내장 |
| **통신** | 시리얼 명령 | XBee 프로토콜 |
| **로봇 수** | 1-2대 | 최대 13대 동시 제어 |
| **액추에이터** | 모터만 | 모터 + 전자석 + 솔레노이드 |
| **Pure Data** | 미지원 | 완전 통합 |
| **제어 주파수** | 100Hz | 100Hz |
| **디스플레이** | 회전수 (0-1) | 각도 (-90° ~ +90°) |
| **NeoKey 기능** | 4개 위치 | 기본 위치 + 정지 |
| **최대 속도** | 15 rev/s | 100 rev/s |
| **위치 정확도** | 0.05 회전 | 0.003 회전 (저속) |

---

## 선택 가이드

### main 브랜치를 선택하세요:
- ✅ 외부 엔코더 사용 (높은 정밀도)
- ✅ 단일 또는 소수 로봇
- ✅ 유선 제어로 충분
- ✅ 간단한 시리얼 인터페이스
- ✅ CANBed FD 보드 사용

### electromagnet-solenoid-control 브랜치를 선택하세요:
- ✅ 다중 로봇 동시 제어 (13대)
- ✅ 무선 제어 필요
- ✅ Pure Data와 통합
- ✅ 전자석/솔레노이드 제어
- ✅ 예술 설치 작품
- ✅ Teensy MicroMod 사용

---

## 공통 기능

### 모든 브랜치에서 제공:
- ✅ 온도 모니터링 (60°C 자동 정지)
- ✅ OLED 디스플레이 (SSD1306)
- ✅ NeoKey 1x4 버튼 제어
- ✅ 긴급 정지 기능
- ✅ 속도 제어 모드 (position = NaN)
- ✅ CAN 통신 (1Mbps)
- ✅ 타임아웃 보호 (15초)
- ✅ I2C 듀얼 버스 (Wire, Wire1)

### 주요 라이브러리:
- mjbots/Moteus@^1.0.2
- ACAN2517FD
- Adafruit_SSD1306
- Adafruit_NeoPixel
- Adafruit_Seesaw

---

## 개발 로드맵

### main 브랜치 향후 계획:
- [ ] 다중 모터 그룹 제어
- [ ] 궤적 추적 기능
- [ ] 설정 파일 시스템
- [ ] 원격 모니터링

### electromagnet-solenoid-control 향후 계획:
- [ ] 브로드캐스트 그룹 기능
- [ ] OSC 프로토콜 지원
- [ ] 웹 인터페이스
- [ ] 시퀀스 레코딩/재생
- [ ] 더 많은 액추에이터 타입

---

## 문서

### 상세 문서 위치:
- **main 브랜치**: `README.md` (루트)
- **electromagnet-solenoid-control**: `BRANCH_electromagnet-solenoid-control.md`

### 추가 문서:
- `MULTIROBOT_README.md`: 다중 로봇 제어 가이드
- `VELOCITY_CONTROL_README.md`: 속도 제어 설명
- `XBEE_PUREDATA.md`: XBee/Pure Data 통합
- `PD_COMPORT_FIX.md`: Pure Data comport 문제 해결

---

## 기술 지원

### 문제 발생 시:
1. 해당 브랜치의 README 확인
2. 시리얼 모니터 디버그 출력 확인
3. I2C 스캐너로 하드웨어 확인
4. GitHub Issues에 보고

### 연락처:
- Repository: seoul-open-media/Moteus_Control
- Branch: electromagnet-solenoid-control (현재)

---

*최종 업데이트: 2025년 12월 1일*
