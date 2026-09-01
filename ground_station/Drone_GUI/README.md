# Drone GUI Ground Station

`guitest.py`는 지도/웨이포인트 UI를 제공하고, `keytest.py`는 NTRIP 수신과
TTGO 시리얼 링크의 2초 반이중 송신 주기를 관리합니다.

## RTCM 정책

- 허용 타입: `1004`, `1006`, `1012`, `1230`
- 타입마다 최신 정상 프레임 하나만 보관
- 동일 타입의 새 프레임은 이전 미전송 프레임을 교체
- 2초마다 타입 순환 방식으로 신선한 프레임 하나만 TTGO에 전달
- 오래된 프레임은 폐기하고 이미 전송한 프레임은 재전송하지 않음
- Waypoint가 대기 중이면 같은 cycle에서 RTCM 다음에 전달

## 실행

Python 3.9 이상에서 의존성을 설치합니다.

```powershell
python -m pip install -r requirements.txt
```

실제 계정과 API 키는 저장소에 넣지 말고 환경변수로 지정합니다.

```powershell
$env:NTRIP_USER = "your-account"
$env:NTRIP_PASSWORD = "your-password"
$env:VWORLD_API_KEY = "your-vworld-api-key"
$env:GCS_TTGO_PORT = "COM8"
python guitest.py
```

필요한 환경변수 목록은 `.env.example`을 참고하세요. 프로그램이 `.env`를
자동으로 읽지는 않으므로 PowerShell, VS Code 실행 설정 또는 운영체제 환경변수로
설정해야 합니다.

## 주요 로그

```text
[RTCM] filter pass types=1004,1006,1012,1230
[CYCLE N] RTCM selected: type=1004, ..., policy=type-round-robin
[RTCM TX] cycle=N, type=1004, queued=... B, then enter RX
```
