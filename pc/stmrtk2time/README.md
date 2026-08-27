# TTGO RTCM 송신 + STM32 ST-LINK RTK 모니터

`stmrtkcompact`와 같은 통신/RTCM/GGA 구조를 사용하면서, CRC24Q가 정상인
`1004/1006/1012/1230`을 하나의 공통 gate로 약 2초마다 한 correction
epoch만 COM8의 TTGO에 전달합니다. 다른 타입과 중간 epoch는 즉시 폐기되며 나중에
재전송되지 않습니다. STM32 펌웨어는 변경하지 않습니다.

동시에 COM7의 STM32 ST-LINK VCP를 열어 LORARTK 디버그 로그와 UM982 GGA를
같은 VSCode 터미널에 표시합니다. 따라서 VSCode에서 `main.py`의 Run 버튼만
눌러도 RTCM 송신과 STM32 로그 확인이 함께 동작합니다.

## Gate 설정

`config.py`에서 다음 값을 변경할 수 있습니다.

```python
RTCM_FORWARD_INTERVAL = 2.0
RTCM_EPOCH_WINDOW = 0.35
```

첫 대상 프레임이 gate를 열면 즉시 전송되고, `RTCM_EPOCH_WINDOW` 동안 뒤따르는
같은 epoch의 나머지 대상 프레임도 즉시 전송됩니다. window가 끝난 후 다음
gate가 열릴 때까지 들어오는 대상 프레임은 저장 없이 `2SEC DROP` 됩니다.

## 설치 및 실행

```powershell
cd "C:\Users\hongt\OneDrive\바탕 화면\stmrtk2time"
py -3.9 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt

python main.py --list-ports
python main.py

# 설정 파일을 바꾸지 않고 1.5초 간격 실험
python main.py --port COM8 --monitor-port COM7 --interval 1.5 --epoch-window 0.35
```

Tera Term이나 다른 프로그램이 COM7 또는 COM8을 사용 중이면 먼저
종료해야 합니다.

## 기타 모드

```powershell
python main.py --monitor-port COM7 --serial-only
python main.py --port COM8 --monitor-port COM7 --rtcm-file sample.rtcm3 --file-rate 1000
python main.py --port COM8 --monitor-port COM7 --save-rtcm captures\suwn_raw.rtcm3
```

`--save-rtcm`은 필터 전 원본을 저장합니다. 파일 재생 모드에서도 동일한 타입
필터와 monotonic 2초 gate를 통과한 프레임만 STM32에 전송됩니다.

## 정상 로그

```text
[NTRIP] RAW RTCM = 690 B/s
[FILTER] TYPE DROP = 60 B/s
[FILTER] 2SEC DROP = 315 B/s
[FILTER] PASS = 315 B/s
[PC->TTGO] TX = 315 B/s

[PASS] 1004 x1  1006 x1  1012 x1  1230 x1
[TYPE DROP] 1019 x14  1020 x11
[2SEC DROP] 1004 x1  1006 x1  1012 x1  1230 x1
```

정상 상태에서는 `PASS == Serial TX`, STM32의 `received == forwarded`,
`CRC errors=0`, `queued_drop=0`이어야 합니다. GGA의 `quality=5`는 RTK FLOAT,
`quality=4`는 RTK FIXED입니다. 누적 절감률은 `[RTCM TOTAL]`에서 확인합니다.
