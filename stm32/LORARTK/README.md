# LORARTK — STM32 드론측 RTCM 수신기

`LORARTK`는 지상 TTGO LoRa가 보낸 RTCM 바이너리를 STM32F722에서 수신하고, 분할된 프레임을 원래 RTCM 프레임으로 재조립한 뒤 UM982 Rover의 보정 입력으로 전달하는 **드론측 수신 전용** 프로젝트다.

이 프로젝트에는 LoRa 송신, ACK/NACK, 재전송 요청, 제어 패킷, 텔레메트리 송신 기능이 없다.

## 전체 데이터 경로

```text
NTRIP caster
  -> PC stmrtk2time (RTCM 수신/출력)
  -> RTKTTGO (USB Serial 입력, LoRa 송신)
  -> SX1276 (드론 LoRa 수신)
  -> STM32F722 LORARTK (순서 검사, 재조립, CRC24Q)
  -> USART2 TX PD5
  -> UM982 보정 입력(RX)
  -> UM982가 RTK 위치 계산
```

UM982의 NMEA 출력은 반대 방향으로 `UM982 TX -> STM32 PD6(USART2 RX)`에 들어온다. STM32는 `$GNGGA`/`$GPGGA`를 분석해 ST-LINK VCP로 출력한다.

## RTKTTGO 호환 패킷

현재 `RTKTTGO/src/main.cpp`의 송신 형식과 동일하다.

### RTCM 프레임이 126 bytes 이하

```text
[0] 0xA1
[1] frame sequence (uint8, 0~255 순환)
[2..] 완전한 RTCM 프레임
```

### RTCM 프레임이 127 bytes 이상

```text
[0] 0xA2
[1] frame sequence (같은 RTCM의 모든 조각에서 동일)
[2] fragment index (0부터 시작)
[3] fragment count
[4..] fragment data (최대 124 bytes)
```

예를 들어 155-byte RTCM은 `128 bytes(4+124)`와 `35 bytes(4+31)`인 두 LoRa 패킷으로 들어온다.

수신기는 fragment를 순서대로만 받는다. 중복 fragment는 무시하고, 누락/역순/fragment count 변경/1.5초 timeout은 해당 재조립을 폐기한다. 완성 후 RTCM preamble, 10-bit payload length, 전체 프레임 길이와 CRC24Q가 모두 맞는 프레임만 UM982에 전달한다. RTCM 바이트를 임의로 버리거나 필터링하지 않는다.

## 하드웨어 연결

### SX1276

| 기능 | STM32 핀 | 설정 |
|---|---:|---|
| EN | PG2 | 출력, High |
| RESET | PD15 | 출력, High |
| DIO0 | PC11 | EXTI15_10, RxDone |
| SCK | PA5 | SPI1 |
| MISO | PA6 | SPI1 |
| MOSI | PA7 | SPI1 |
| NSS/CS | PD14 | 출력, idle High |

### UM982

| 연결 | STM32 핀 | 의미 |
|---|---:|---|
| STM32 -> UM982 | PD5 / USART2_TX | RTCM 보정 데이터, UM982 RX에 연결 |
| UM982 -> STM32 | PD6 / USART2_RX | NMEA 출력, UM982 TX에 연결 |
| 공통 | GND | 반드시 공통 접지 |

USART2는 `115200, 8 data bits, no parity, 1 stop bit`이다. PD6 수신은 `DMA1 Stream5 Channel4 + Receive-to-idle`로 동작한다.

### 디버그 터미널

NUCLEO 보드의 ST-LINK VCP에 연결된 USART3를 사용한다.

```text
PD8  = USART3_TX
PD9  = USART3_RX
115200 8N1
```

현재 PC에서는 COM7을 열면 된다. 장치 번호가 바뀌면 Windows 장치 관리자에서 `STMicroelectronics STLink Virtual COM Port` 번호를 확인한다.

### RSSI 측정 출력(PG14)

원본 `lora_drone`의 Matlab/수신감도 측정용 출력과 동일하게 USART6 TX인 **PG14**에서도 `115200 8N1` 로그를 출력한다. PG14는 ST-LINK COM7과 다른 UART이므로, PC에서 보려면 `PG14 -> USB-UART 어댑터 RX`, `GND -> 어댑터 GND`로 연결하고 해당 어댑터의 COM 포트를 연다.

```text
R,<tick_ms>,<current_rssi_dbm>
P,<tick_ms>,<packet_rssi_dbm>,<snr_x100>,<crc_ok>,<packet_hex>
T,<tick_ms>,<sequence>,<rtcm_type>,<frame_length>,<crc_ok>,<fragment_count>,<forwarded>,<status>
```

- `R`은 패킷이 없을 때도 50 ms마다 출력되는 현재 채널 RSSI다.
- `P`는 LoRa packet 수신 때 출력되며, `crc_ok`가 `1`이면 뒤에 TTGO packet 전체가 hex로 붙는다.
- `T`는 단일 또는 분할 RTCM을 완성한 뒤 출력되는 최종 결과이며 MATLAB 로그창에서 RTCM type, 전체 길이, CRC와 UM982 전달 여부를 판정하는 데 사용한다.
- `snr_x100=-125`는 `-1.25 dB`를 뜻한다.
- RTCM 처리를 위해 이미 읽은 FIFO 복사본을 출력하므로 RSSI 점검 기능이 재조립 데이터를 다시 소비하지 않는다.

## LoRa 무선 설정

원본 `C:\Users\hongt\lora_drone`의 레지스터 설정과 주석을 기반으로 유지했다.

| 항목 | 값 |
|---|---|
| Frequency | 922.1 MHz (`FRF E6 86 66`) |
| Spreading Factor | SF7 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| Header | Explicit |
| Payload CRC | ON |
| PA | PA_BOOST, 17 dBm |
| OCP | 140 mA |
| RX mode | Continuous |
| DIO0 | RxDone |
| RX FIFO base | `0x00` |

수신 완료 시 FIFO를 SPI burst로 먼저 복사하고 지연 없이 RX Continuous로 복귀한다. 로그, 재조립, CRC 계산은 RX 복귀 후 수행하므로 다음 패킷을 받을 수 없는 시간을 최소화한다.

## 주요 소스

- `Core/Src/lora.c`: SX1276 레지스터 설정, FIFO binary burst 수신, RSSI/SNR
- `Core/Src/lorartk.c`: TTGO packet 해석, sequence 검사, fragment 재조립, CRC24Q, RTCM type/통계
- `Core/Src/rtk_bridge.c`: UM982 USART2 송수신 큐/DMA, USART3 비동기 로그, GGA 분석
- `Core/Src/main.c`: Cube HAL 초기화와 메인 루프

## COM7 로그 예

```text
=== LORARTK STM32 Drone Receiver ===
[RADIO] SX1276 version=0x12, 922.1 MHz, SF7, BW125, CR4/5, Explicit, CRC ON
[FRAG] Sequence=37 Index=0/2 Data=124
[FRAG] Sequence=37 Index=1/2 Data=31
[REASM] Complete sequence=37 fragments=2 length=155
[RTCM] Type=1077 Frame Length=155 CRC=OK
[UM982 TX] Queued RTCM type=1077, 155 bytes
[UM982 RAW] $GNGGA,...
[GGA] UTC=... Lat=... Lon=... Alt=... m Quality=4 (RTK FIXED) Sats=... HDOP=... DiffAge=... s
```

GGA quality 표시는 다음과 같다.

| quality | 출력 |
|---:|---|
| 0 | INVALID |
| 1 | GNSS FIX |
| 2 | DGNSS/DGPS |
| 4 | RTK FIXED |
| 5 | RTK FLOAT |
| 6 | ESTIMATED |

1초마다 LoRa packet/bytes, 복원 RTCM frame/bytes, UM982 전달량, CRC/재조립/sequence/drop 통계를 출력한다. 10초마다 누적 합계와 RTCM 1005/1006/1077/1087/1097/1127/1230/기타 type 개수를 출력한다.

## 빌드 및 실행

1. STM32CubeIDE에서 `File -> Import -> Existing Projects into Workspace`를 선택한다.
2. `C:\Users\hongt\STM32CubeIDE\workspace_2.1.1\LORARTK`를 선택한다.
3. `Debug` configuration을 빌드한다.
4. NUCLEO에 프로그램한 뒤 COM7을 `115200 8N1`로 연다.
5. TTGO를 켜기 전에 시작 로그에서 SX1276 version이 보통 `0x12`인지 확인한다.
6. PC의 `stmrtk2time`과 `RTKTTGO`를 실행한다.
7. `[RTCM] ... CRC=OK`, `[UM982 TX]`, GGA quality 변화 순서로 확인한다.

## 현장 점검 순서

1. LoRa packet 수가 증가하지 않으면 주파수/SF/BW/CR, 안테나, DIO0와 CS 배선을 확인한다.
2. packet은 증가하지만 재조립 오류가 늘면 TTGO 패킷 형식과 sequence/fragment log를 확인한다.
3. CRC 오류가 늘면 LoRa payload CRC 설정과 송신측이 RTCM 원본 바이트를 변경하지 않는지 확인한다.
4. `[UM982 TX]`는 나오지만 RTK가 되지 않으면 PD5 -> UM982 RX, 공통 GND, UM982 Rover/RTCM 입력 포트 설정을 확인한다.
5. GGA가 안 나오면 UM982 TX -> PD6, baud rate와 UM982 NMEA 출력 설정을 확인한다.

`Debug/LORARTK.elf`와 `Debug/LORARTK.hex`는 CubeIDE Debug 빌드 산출물이다.

PG14 RSSI/RTCM 모니터는 `Tools/LORARTK_RTCM_Monitor.m`을 MATLAB에서 실행한다. 기존 RSSI/SNR/airtime 그래프 동작은 유지하며, 두 번째 창에서 TTGO `0xA1` 단일 packet과 `0xA2` fragment를 재조립해 RTCM type, 길이, CRC24Q, UM982 전달 여부와 전체 RTCM hex를 표시한다.
