# JISANG — ESP32-S3 드론측 LoRa RTCM 수신기

STM32 `LORARTK`에서 검증한 기능을 ESP32-S3용으로 옮긴 **VS Code + PlatformIO 프로젝트**다. VS Code에서 이 폴더 전체를 열어 빌드·업로드·시리얼 모니터를 실행한다.

## 구현 기능

- SX1276 LoRa 수신: 922.1 MHz, SF7, BW 125 kHz, CR 4/5, Explicit header, payload CRC ON
- 지상 송신기 패킷 `A1`(단일 RTCM), `A2`(분할 RTCM) 호환
- 지상국 전용 packet 조건 검사 후 통과 (`A1/A2` marker, RTCM3 header, fragment 문맥)
- sequence 검사, fragment 순서 검사, 1초 timeout, 최대 1029-byte 재조립
- RTCM3 preamble/길이/CRC24Q 검증 후에만 UM982 UART로 전달
- UM982 `$GNGGA`/`$GPGGA` 10 Hz 수신 및 RTK quality/위도/경도/고도
  `G,tick,utc,lat,lon,alt,quality,sats,hdop,diff_age,status` USB 로그 출력
- 부팅 시 `GPGGA 0.1`을 보내 UM982 보정 위치를 10 Hz로 출력하도록 설정
- 8 KB UART RX buffer와 1초 단위 `UM982 B/s`, 전체 문장 수, 실제 GGA Hz,
  마지막 GGA age 및 line overflow 감시
- MATLAB `rtkmathlab.m`과 동일한 `R/P/T` 디버그 형식 출력

## 사진 기준 LoRa 배선

| LoRa | ESP32-S3 GPIO |
|---|---:|
| CSn | 8 |
| CLK | 12 |
| MISO | 13 |
| MOSI | 11 |
| EN | 6 |
| RST | 14 |
| G0 / DIO0 | 38 |
| VDD5V | 보드 5V |
| GND | 보드 GND |

LoRa 모듈 전원 입력만 사진대로 5V에 연결한다. ESP32 GPIO 신호는 3.3V이며 GPIO에 5V를 직접 넣으면 안 된다. 모든 장치의 GND를 공통으로 연결한다.

## UM982 기본 배선

사진에 UM982 핀이 없어서 다음 핀을 기본값으로 잡았다. 실제 배선이 다르면 `include/config.h`의 두 상수만 바꾼다.

| 연결 | ESP32-S3 |
|---|---:|
| UM982 TX → ESP32 RX | GPIO17 |
| ESP32 TX → UM982 RX | GPIO18 |
| GND | 공통 GND |

UART는 115200 8N1이다. GPIO18에서 RTCM 원본이 UM982로 나가고 GPIO17에서 보정된 GGA가 들어온다.

## VS Code / PlatformIO 실행

1. VS Code에서 `File → Open Folder`로 `C:\Users\hongt\OneDrive\바탕 화면\JISANG`을 연다.
2. 확장 설치 안내가 뜨면 `PlatformIO IDE`를 설치한다.
3. 하단 PlatformIO ✓ 버튼으로 Build한다.
4. → 버튼으로 ESP32-S3에 Upload한다.
5. 플러그 모양 메뉴의 `Monitor`를 실행하거나 PlatformIO 터미널에서 아래 명령을 실행한다.

```powershell
pio run
pio run -t upload
pio device monitor -b 115200
```

보드는 `platformio.ini`에서 `esp32-s3-devkitc-1`, 현재 ESP32-S3 Native USB
포트인 COM9로 설정했다. `ARDUINO_USB_MODE=1`과
`ARDUINO_USB_CDC_ON_BOOT=1`도 활성화되어 `Serial` 로그가 COM9로 출력된다.
COM 번호가 바뀌면 `upload_port`, `monitor_port`와 MATLAB의 `port`를 함께 바꾼다.

시작 로그의 `SX1276 version=0x12`를 확인한다. `0x00`/`0xFF`이면 전원, CS, SPI 또는 RESET 배선을 점검한다. 외부 Arduino 라이브러리는 필요 없고 ESP32 Arduino core의 `Arduino`, `SPI`, `HardwareSerial`만 사용한다.

## MATLAB 연결

ESP32 USB 포트를 Windows 장치 관리자에서 확인한 뒤 기존 파일

`C:\Users\hongt\OneDrive\바탕 화면\rtkmathlab.m`

상단의 다음 줄만 ESP32 COM 번호로 변경한다.

```matlab
port = "COM번호";
baud = 115200;
```

코드가 STM과 동일하게 출력하는 형식:

```text
R,<tick_ms>,<current_rssi_dbm>
P,<tick_ms>,<packet_rssi_dbm>,<snr_x100>,<crc_ok>,<packet_hex>
T,<tick_ms>,<sequence>,<rtcm_type>,<frame_length>,<crc_ok>,<fragment_count>,<forwarded>,<status>
```

따라서 기존 MATLAB의 RSSI/SNR 그래프와 RTCM 재조립·CRC·HEX 로그를 그대로 사용할 수 있다. 같은 COM 포트를 Arduino Serial Monitor와 MATLAB이 동시에 열 수 없으므로 MATLAB 실행 전 Serial Monitor를 닫아야 한다.

## 정상 로그 순서

```text
[RADIO] SX1276 version=0x12 ...
P,...
[FRAG] Sequence=... Index=.../... Data=...
[REASM] Complete ...
[RTCM] Type=... Frame Length=... CRC=OK
[UM982 TX] Queued RTCM ...
T,...,1,...,1,OK
G,...,...,37.12345678,127.12345678,42.1,4,24,0.7,0.5,RTK FIXED
[1s] ... UM982=... B/s,... lines/s GGA=10 Hz(OK),age=... ms,NMEAovf=0
```

`T`의 `forwarded=1`은 ESP32가 검증된 RTCM을 UM982 UART에 기록했다는 뜻이다. 실제 보정 적용은 이후 GGA의 `Quality=4`(RTK Fixed) 또는 `Quality=5`(RTK Float), 그리고 `DiffAge`로 확인한다.

`GGA=10 Hz(OK)`가 계속 보이면 ESP32가 보정 위치를 실제 10 Hz로 읽는 것이다.
`WARN` 또는 `age` 증가가 반복되면 UM982 포트 설정, 배선, baud를 확인한다.
UM982가 여러 NMEA 메시지를 모두 10 Hz로 보내도 UART에서는 전부 읽지만, USB
병목을 막기 위해 기본 로그에는 GGA만 해석해서 표시한다. GGA 원문도 함께 보고
싶으면 `include/config.h`의 `UM982_PRINT_RAW_GGA`를 `true`로 바꾼다.

## 지상국 packet 필터

현재 실제 `RTKTTGO` 무선 packet은 아래 형식이며 `lora_drone`에 정의만 되어 있던 `ID_GROUND=0xFF`를 LoRa payload에는 싣지 않는다.

```text
A1 SEQ D3...                         (단일 RTCM)
A2 SEQ INDEX COUNT D3...             (첫 fragment)
A2 SEQ INDEX COUNT ...               (후속 fragment)
```

그래서 ESP32 수신기는 존재하지 않는 `0xFF`를 억지로 요구하지 않고 다음 조건을 모두 검사한다.

1. packet marker가 지상 RTKTTGO 전용 `0xA1` 또는 `0xA2`
2. 단일 packet/첫 fragment가 정상 RTCM3 `0xD3` header로 시작
3. 후속 fragment가 이미 통과한 동일 sequence/count의 바로 다음 index
4. 완성된 RTCM 길이와 CRC24Q가 정상

하나라도 맞지 않으면 `[FILTER] DROP`으로 버리며 UM982로 전달하지 않는다. `[FILTER] PASS` 뒤에도 최종 CRC24Q 검증을 통과해야 `T,...,OK`가 나온다. 이는 현재 지상 송신 코드를 바꾸지 않으면서 적용할 수 있는 packet 구분이다. 송신 장치 자체를 인증하는 보안 기능이 필요한 경우에는 지상 TTGO와 드론 양쪽 protocol에 별도의 ID/인증값을 함께 추가해야 한다.
