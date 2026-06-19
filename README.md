# Moteus Motor Control System
## 브랜치: Ahae_MoCA_Busan_2026

## 개요
Moteus R4.11 모터 컨트롤러 1개를 CANBed FD 보드로 제어합니다.  
AS5600 외부 앤코더(AUX2)로 절대 위치를 피드백받으며, 속도 기반 제어와 안전 기능을 포함합니다.  
**무선 XBee 통신은 비활성화**되어 있으며, Pure Data(PD)에서 USB 유선 시리얼로 명령을 수신합니다.

## 하드웨어
- **모터 컨트롤러**: Moteus R4.11 × 1 (ID: 2)
- **통신**: CANBed FD (Longan Labs) — CAN bus 1Mbit
- **앤코더**: AS5600 마그네틱 앤코더 (AUX2 포트)
- **마이크로컨트롤러**: Teensy MicroMod (teensymm)
- **디스플레이**: Adafruit 128×64 OLED (SSD1306) — I2C bus 1 (Wire1), 주소 0x3C
- **작동기**: 전자석(EM) 3개 + 솔레노이드 1개
- **호스트 소프트웨어**: Pure Data (PD) — USB Serial 115200bps

## 파일 구조

### `src/config.h` — 설정 파일
로봇마다 반드시 수정해야 할 항목:

| 항목 | 기본값 | 설명 |
|------|--------|------|
| `ROBOT_ID` | `6` | 로봇 번호 (1~13), 각 로봇마다 고유하게 설정 |
| `MOTOR_OFFSET` | `0.0` | 0도 교정 오프셋 (단위: 회전수) |
| `ENCODER_CENTER` | `0.5` | 앤코더 물리적 중앙값 (0.0~1.0) |
| `MOTOR2_DIRECTION` | `1.0` | 방향 반전 시 `-1.0` |

제어 파라미터:

| 항목 | 값 | 설명 |
|------|----|------|
| `POSITION_TOLERANCE` | `0.01` | 도달 판정 허용 오차 (≈3.6도) |
| `MAX_VELOCITY_FAST` | `2.0` | 최대 속도 (rev/s) |
| `SLOW_ZONE` | `0.06` | 감속 시작 거리 (≈21.6도) |
| `BRAKE_ZONE` | `0.06` | 브레이크 구간 |
| `TIMEOUT_MS` | `15000` | 이동 타임아웃 (15초) |

---

### `src/xbee.h` / `src/xbee.cpp` — 유선 시리얼 제어 (XBee 비활성화)
- **XBee (Serial4) 완전 비활성화** — 주석 처리됨
- `updateWiredSerial()` : USB Serial로 PD 바이너리 패킷 수신 및 처리

**패킷 프로토콜:**
```
[0xFF] [0xFF] [R1_MSB] [R1_LSB] ... [R13_MSB] [R13_LSB]
총 28바이트 (헤더 2 + 로봇 13대 × 2바이트)
```

**값 디코드:**
```
value = (MSB × 256) + LSB
digit1 = (value / 1000) % 10  → 속도  (0=0.3rev/s ~ 9=2.0rev/s)
digit2 = (value /  100) % 10  → 무시
digit3 = (value /   10) % 10  → 각도  (0=-90° ~ 9=+90°)
digit4 =  value         % 10  → 작동기 (0=없음 / 1=전자석 / 2=솔레노이드)
```

**안전장치:**
- `peek() != 0xFF` 이면 텍스트 명령 핸들러로 패스 (충돌 없음)
- 100ms 이내 28바이트 미수신 시 버퍼 자동 flush (데드락 방지)
- 잘못된 헤더 수신 시 버퍼 flush

---

### `src/motor_control.h` / `src/motor_control.cpp` — 모터 제어

#### `moveToEncoderPosition(target_ext1, target_ext2, max_velocity)`
메인 위치 제어 함수:
1. 현재 앤코더 위치 쿼리
2. 오차 계산 (wrap-around 처리 포함)
3. 100Hz 제어 루프:
   - 오차 > SLOW_ZONE → 최대 속도
   - 오차 ≤ SLOW_ZONE → 비례 감속
   - 오차 ≤ BRAKE_ZONE → 브레이킹
   - 오차 ≤ TOLERANCE → 정지 확인 후 완료
4. 오버슈트 감지 시 브레이킹 모드 전환
5. 15초 타임아웃

#### `checkMotorTemperature()`
- 250ms마다 온도 확인
- 60°C 초과 시 → 전체 정지 + 무한 루프 (리셋 필요)

---

### `src/solenoid.h` / `src/solenoid.cpp` — 작동기 제어

| 함수 | 동작 |
|------|------|
| `triggerElectromagnets()` | EM1+EM2+EM3 50ms 순차 ON |
| `triggerSolenoid()` | 솔레노이드 50ms ON |
| `triggerTypeA_2()` | 솔레노이드만 ON (digit4=2) |
| `updateSolenoid()` | 타이밍 관리 (매 루프 호출) |

---

### `src/display.h` / `src/display.cpp` — OLED 디스플레이
- 128×64 SSD1306, I2C Wire1, 주소 0x3C
- 모터 위치·온도 실시간 표시
- `displayDebug("msg")` : 디버그 메시지 3초 표시
- 온도 경고: 50°C → "WARM!" / 60°C → "HOT!!!" (반전 표시)

---

### `src/main.cpp` — 메인 루프

**setup():**
1. `Serial.begin(115200)` + `Serial.setTimeout(20)`
2. OLED 초기화
3. 솔레노이드 초기화
4. SPI / CAN bus 초기화 (1Mbit)
5. 모터 정지 (fault 클리어)

**loop():**
1. 모터 쿼리 (20Hz)
2. 온도 체크
3. OLED 업데이트
4. `updateWiredSerial()` — PD 바이너리 패킷 처리
5. `updateSolenoid()` — 작동기 타이밍
6. 텍스트 명령 처리 (peek ≠ 0xFF 일 때만)

---

## 시리얼 텍스트 디버그 명령
Serial Monitor (115200baud) 에서 직접 입력:

| 명령 | 동작 |
|------|------|
| `0` | 모터 정지 (긴급 정지) |
| `1` | Motor2 → 0° |
| `2` | Motor2 → -45° |
| `3` | Motor2 → +45° |
| `4` | Motor2 → +90° |
| `5` | 전자석 ON |
| `6` | 솔레노이드 ON |

> PD 운영 중에도 텍스트 명령 사용 가능 (프로토콜 충돌 없음)

---

## 다운로드 및 설치

```bash
# 브랜치 클론
git clone -b Ahae_MoCA_Busan_2026 https://github.com/seoul-open-media/Moteus_Control.git

# 이미 클론된 경우
git fetch origin
git checkout Ahae_MoCA_Busan_2026
```

**업로드 전 필수 수정:** `src/config.h` 에서 `ROBOT_ID` 를 각 로봇에 맞게 변경

```bash
# PlatformIO로 빌드 & 업로드
platformio run --target upload
```

---

## 문제 해결

| 증상 | 원인 / 해결 |
|------|------------|
| 모터가 안 움직임 | CAN 버스 연결 확인 / OLED 온도 확인 (60°C 초과 시 리셋 필요) |
| 각도가 맞지 않음 | `config.h` 의 `MOTOR_OFFSET` 조정 |
| 방향이 반대 | `MOTOR2_DIRECTION = -1.0` 으로 변경 |
| PD 명령 무반응 | `ROBOT_ID` 가 PD 패치의 로봇 번호와 일치하는지 확인 |
| 텍스트 명령 무반응 | PD가 불완전 패킷 전송 중일 수 있음 — PD 패치 정지 후 재시도 |
| OLED 안 보임 | Wire1 SDA/SCL 연결 및 주소 0x3C 확인 |
- Look for initialization messages in serial monitor
- Check for I2C address conflicts

### STOP button doesn't work during movement
- This should work - STOP is checked every 50ms
- Check serial monitor for "STOP button pressed during movement!"
- Verify NeoKey library is up to date
- Ensure no I2C communication errors

### Display shows old values
- Motors are queried every 50ms automatically
- If values frozen, check CAN bus communication
- Verify motor controllers are responding
- Check for timeout errors in serial monitor

## Technical Notes

### Encoder Reading
- AS5600 provides absolute position (0-1 revolution)
- Connected via AUX2 port on Moteus
- 21:1 gear ratio between motor and encoder
- Readings in external encoder revolutions

### CAN Bus Configuration
- 1Mbit bus speed
- Custom FIFO sizes for ATmega32u4 (limited RAM)
- No BRS (Bit Rate Switching) for Arduino compatibility

### I2C Bus Architecture
- **Wire (I2C bus 0)**: NeoKey 1x4 at 100kHz
  - Address: 0x30
  - Slower clock for reliable Seesaw communication
- **Wire1 (I2C bus 1)**: OLED display at 400kHz
  - Address: 0x3C
  - Faster clock for display updates
- **Separation Reason**: Prevents I2C conflicts and timing issues
- **I2C Scanner**: Runs at startup to verify device detection

### Memory Optimization
- Uses 16-bit integer format for CAN messages
- Flash strings with F() macro to save RAM
- Static variables for timing (no global pollution)
- Circular buffer for debug messages (6 messages max)

### Update Rates
- **Motor Control Loop**: 100Hz (10ms period)
- **Motor Queries**: 20Hz (50ms period)
- **Display Update**: 10Hz (100ms period)
- **Temperature Check**: 4Hz (250ms period)
- **NeoKey Check in Loop**: Every iteration (~1ms)
- **NeoKey Check During Movement**: 20Hz (50ms period)

### Libraries Used
- `Moteus` v1.0.2 - Motor controller communication
- `ACAN2517FD` - MCP2517 CAN controller driver
- `Adafruit_SSD1306` v2.5.7 - OLED display driver
- `Adafruit_GFX` v1.11.3 - Graphics library
- `Adafruit_NeoPixel` v1.12.0 - LED control for NeoKey
- `Adafruit_Seesaw` v1.7.0 - NeoKey I2C interface
