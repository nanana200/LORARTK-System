# LORARTK System

NTRIP RTCM 데이터를 PC에서 TTGO LoRa 송신기로 전달하고, 드론의 STM32가 LoRa로 수신해 UM982 Rover에 보정 데이터로 입력하는 RTK 테스트 시스템입니다.

## 데이터 흐름

```text
GNSS 기준국 / NTRIP caster
  -> PC stmrtk2time
  -> COM8 / TTGO LoRa 송신기
  -> LoRa
  -> STM32F722 LORARTK
  -> USART2 / UM982 RTCM 입력
  -> UM982 RTK 위치 계산
  -> UM982 GGA
  -> STM32
  -> COM7 / ST-LINK VCP
  -> PC 로그
```

## 저장소 구성

```text
pc/stmrtk2time/   NTRIP 수신, RTCM 필터링, TTGO 전송, STM32 로그 모니터
stm32/LORARTK/    SX1276 LoRa 수신, RTCM 재조립/CRC24Q, UM982 UART 브리지
ground_station/Drone_GUI/  GUI + KEYTEST NTRIP/TTGO 2초 송신 스케줄러
drone/JISANG/              ESP32-S3 LoRa 수신, RTCM 재조립, UM982 브리지
```

## 현재 GUI + ESP32 구성

현재 통합 테스트 구성은 `ground_station/Drone_GUI`와 `drone/JISANG`입니다.

```text
NTRIP caster
  -> Drone_GUI/keytest.py
  -> RTCM 1004/1006/1012/1230 타입별 최신 프레임
  -> 2초마다 신선한 프레임 1개 선택
  -> TTGO A1/A2 LoRa 송신
  -> ESP32-S3 JISANG 재조립 및 CRC24Q 검사
  -> UM982 RTCM 입력
  -> UM982 GGA
  -> ESP32 USB-UART 디버그
```

GUI 실행 방법과 환경변수는 `ground_station/Drone_GUI/README.md`를 참고하세요.

## PC 프로그램 실행

Python 3.9 이상에서 실행합니다.

```powershell
cd pc\stmrtk2time
python -m pip install -r requirements.txt

$env:NTRIP_USER = "your-account"
$env:NTRIP_PASSWORD = "your-password"
python main.py
```

기본 포트는 다음과 같습니다.

- `COM8`: TTGO RTCM 송신
- `COM7`: STM32 ST-LINK 디버그/GGA 모니터
- `115200 8N1`

환경에 따라 포트가 다르면 다음처럼 지정할 수 있습니다.

```powershell
python main.py --port COM8 --monitor-port COM7
```

실제 NTRIP 계정정보는 저장소에 포함하지 않습니다. 반드시 환경변수나 실행 인자로 제공하세요.

## STM32 프로젝트

`stm32/LORARTK`를 STM32CubeIDE에서 Existing Projects into Workspace로 가져와 `Debug` 구성으로 빌드합니다.

주요 기능:

- SX1276 LoRa 연속 수신
- TTGO `0xA1` 단일 RTCM 및 `0xA2` fragment 형식 지원
- 순서 검사와 분할 프레임 재조립
- RTCM 길이 및 CRC24Q 검사
- 정상 RTCM만 UM982 USART2로 전달
- UM982 `$GNGGA`/`$GPGGA` 수신
- ST-LINK USART3 디버그 로그
- PG14 USART6 RSSI/SNR/RTCM 분석 로그

세부 핀 연결과 로그 형식은 `stm32/LORARTK/README.md`를 참고하세요.

## 주의

- UM982는 Rover 모드와 RTCM 입력 포트가 미리 설정되어 있어야 합니다.
- GGA `quality=5`는 RTK FLOAT, `quality=4`는 RTK FIXED입니다.
- 빌드 산출물, RTCM 캡처, Python 캐시 및 NTRIP 자격증명은 Git에서 제외됩니다.
