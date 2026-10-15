# Quadrotor Racing Prototype

RotorPy를 물리 계산에 사용하고, 학생 제어기와 대회 판정을 분리한 작은 prototype입니다.
기체와 벌점 수치는 대회 최종 규칙이 아닙니다.

2026-10-15 로컬 검증: 기본 4게이트 17.31초 완주, 작은 외란 조건 17.99초 완주,
실제 macOS 창 표시/무화면 실행의 상태·판정·점수 일치, 총 29개 테스트 통과.
정확한 조건과 제한은 [VALIDATION.md](VALIDATION.md)를 참고하세요.

## 실행

프로젝트 폴더에서 실행합니다. 기본 안내는 conda이며, Python 3.12.12와 검증한 패키지 버전을 고정합니다.

```bash
cd /Users/oxcarxierra/aail_workspace/aoc-drone-racing
conda env create -f environment.yml
conda activate aoc-drone-racing
```

venv를 선호하면 Python 3.12를 설치한 뒤 아래 방법을 사용할 수 있습니다.

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
python -m pip install --no-deps -e .
```

활성화된 conda 또는 venv 환경에서 실행합니다.

```bash
quad-race run --controller controller.py --log outputs/nominal.json
quad-race run --controller controller.py --render
quad-race replay outputs/nominal.json --show
quad-race replay outputs/nominal.json --output outputs/replay.gif
python -m pytest
```

화면 없는 환경에서 그림만 만들 수 있습니다.

```bash
quad-race run --render-to outputs/nominal.png
quad-race run --wind 0.5 --mass-error 0.05 --seed 17 --log outputs/perturbed.json
quad-race run --controller examples/hover.py --max-time 2
```

10게이트 랜덤 트랙 `tracks/ten_gates.json`(seed 4)은 기본 제어기로 약 50.8초가 걸리므로 제한 시간을 늘려 실행합니다.
게이트 yaw는 접근 방향을 따르며 구간마다 최대 ±60° 회전합니다. 다른 seed와 게이트 수로 새 트랙을 만들 수 있습니다.

```bash
quad-race run --track tracks/ten_gates.json --max-time 90 --log outputs/ten_gates.json
quad-race make-track tracks/random_seed7.json --gates 10 --seed 7
```

## 학생이 구현하는 부분

`controller.py`의 `Controller.reset(config)`와 `Controller.act(observation)`입니다.
초기화 때 전체 트랙과 nominal 기체 정보를 받고, 매 0.02초마다 관측을 받아 `Action`을 반환합니다.
`Action(thrust_N, body_rates_rad_s)`는 총추력 [N]과 body frame 각속도 명령 [rad/s]입니다.
기본 제어기는 게이트 앞의 접근점과 통과점을 위치 추종하는 보수적 예제이며 최적제어 해법이 아닙니다.
`quad_race.helpers.track_position`은 사용해도 되고 대체해도 됩니다.

- World: 오른손 좌표계, z-up, 중력 `(0, 0, -9.81)` m/s².
- Quaternion: `(x, y, z, w)`, body에서 world로 회전.
- 관측: 위치, world 속도, 자세, body 각속도, 시각, 다음 게이트 번호. 센서 잡음 없는 상태 관측입니다.
- 공개 모델: RotorPy 3.0.0 Hummingbird, nominal 질량 0.5 kg, body-rate P gain 25/s.
- 총추력 명령: 0–15 N, 각속도 명령: 축별 ±3 rad/s. 초과/NaN/오형식 명령은 즉시 실패입니다.
- 명령은 제어 주기 동안 유지하며 물리 스텝은 0.002초입니다. 각 물리 스텝에서 공통 rate controller와 모터 할당을 계산합니다.
- 실제 로터별 추력 포화와 1차 모터 지연은 RotorPy가 처리합니다. 명령 총추력과 실현 총추력은 다를 수 있습니다.
- 초기 상태는 공중 호버입니다. 이륙·착륙과 종료 속도 제한은 현재 과제 범위에 없습니다.

## 판정과 임시 점수

게이트의 `width_m`과 `height_m`은 프레임 안쪽 개구부 크기입니다. yaw로 정한 local +x 방향으로
게이트 평면을 통과해야 하며, 기체 반경 0.28 m를 제외한 개구부 안에 교점이 있어야 합니다.
기체는 구로 근사하고, 프레임/장애물과의 충돌은 반경만큼 확장한 box에 대한 이동 선분 검사로
보수적으로 판정합니다. 물리 스텝 사이 경로는 선형 근사이며 고속/곡선 비행의 정확한 swept volume 보장은 아닙니다.

- 순서대로 마지막 게이트 통과: `finished`.
- 미완료 게이트 역방향 통과: `reverse_gate`; 순서 건너뛰기: `skipped_gate`.
- 충돌: `collision`; 지상/경계 접촉: `out_of_bounds`; 제한 시간 초과: `timeout`.
- 제어기 예외/잘못된 입력/시뮬레이션 오류는 별도 상태와 오류 메시지로 기록합니다.
- 충돌과 위반은 첫 이벤트에서 종료합니다. 복구/재시작 정책은 아직 구현하지 않았습니다.
- 마지막 평면 통과 시각은 물리 스텝 내 선형 보간값입니다.
- 하중은 `n = ||dv/dt - gravity_world|| / g`로 계산하므로 호버는 약 1g입니다.
- 임시 점수는 `비행시간 + 2 * integral(max(0, n - 3)^2 dt) + 실패 시 30초`입니다.
- **미완주 점수는 진단용입니다. 낮은 미완주 점수가 완주보다 좋은 순위가 되어서는 안 됩니다.**
- gate 순서, 허용 영역, 하중 등 최종 대회 규칙과 계수는 배포 전에 확정해야 합니다.

설치 버전은 `requirements.txt`와 `requirements-dev.txt`에 기록합니다. conda는 Python 환경을 만들고,
패키지 설치는 pip로 통일합니다. 새 패키지가 필요하면 이 파일들과 `pyproject.toml`을 함께 관리하세요.

## 로그와 재현성

로그는 공개 설정, scenario, 실제 질량 배율, smooth gust 위상, 제어기 SHA256, 주요 패키지 버전,
상태/입력/이벤트/점수와 계산 시간을 저장합니다. 동일 seed와 코드·환경에서는 동일 상태와 판정을 기대합니다.
벽시계 시간과 제어기 계산 시간은 비행시간과 별개이며 실행마다 달라집니다.
렌더러는 관측 복사본과 로그만 읽으며 물리 계산/점수에 관여하지 않습니다.
화면은 전체 트랙과 움직이는 기체를 보여주는 단일 3D 화면입니다. RotorPy의 `Quadrotor` mesh를 사용해
실제 크기의 4개 로터, arm, 중심 기체를 그리고 로그의 quaternion으로 회전시킵니다.
별도 자세 확대 패널은 표시하지 않습니다. 기체의 빨강 표시는 body +x, 초록 표시는 body +z입니다.
기체 그림은 충돌 판정에 사용하지 않습니다. Matplotlib 기반이며 고품질 게임 렌더러는 아닙니다.
RotorPy 3.0의 mesh가 배열 shape을 직접 바꾸므로 입력을 복사해서 전달합니다. 고정한 NumPy 2.5에서
이 동작의 deprecation notice만 mesh 호출 안에서 제한적으로 숨기고, 다른 경고는 유지합니다.
`--render`는 실시간 속도를 보장하는 모드가 아니라 시뮬레이션 중 화면 표시입니다.

현재 외란은 norm이 지정 풍속 이하인 부드러운 sine wind이고, 질량 배율은 매 episode 처음에
`[1-error, 1+error]`에서 추출합니다. prototype의 지원 범위는 바람 0–2 m/s, 질량 오차 0–20%입니다.
20 m/s 돌풍, 관성/추력 계수 오차, 센서 잡음은 아직 검증하지 않았습니다. 학생은 nominal 모델만 받습니다.
지금은 연습용으로 실제 realization도 로그에 남깁니다. 공식 평가에서는 접근 가능한 정보를 따로 관리해야 합니다.

## TA 개발 경계

| 모듈 | 책임 |
|---|---|
| `api.py` | 학생이 의존할 입력·출력 계약 |
| `plant.py` | RotorPy 연결과 기체 파라미터 |
| `track.py`, `rules.py` | 트랙과 방향·순서·이동 구간 충돌 검사 |
| `env.py` | 시계, 외란, 제어기 호출, 판정·임시 점수·로그 |
| `replay.py` | 화면과 로그 재생 |
| `cli.py` | 실행 명령 |

이 prototype은 **신뢰하는 제어기를 같은 프로세스에서 호출**합니다. 공식 제출 실행용 보안 경계가 아닙니다.
무한 루프/과도한 메모리를 막는 프로세스 격리와 hard timeout, 초기 계획·계산 시간 제한, 제출 수집,
비공개 시나리오와 순위 집계는 후속 구현입니다. `reset`/`act` 시간은 현재 측정만 합니다.
학생이 외부 파일을 사용하지 않는 single-file 제출을 기본 가정하며 추가 의존성 정책은 아직 미확정입니다.

RotorPy의 설치 의존성은 `plant.py` 내부 구현과 별도로 패키지 의존성에 고정해 둡니다.
학기 중 upstream 최신 버전을 자동으로 따라가지 않고, 다음 학기 업데이트는 검증 후 진행하는 편이 좋습니다.

## 출처와 라이선스

- RotorPy: https://github.com/spencerfolk/rotorpy (MIT, Copyright 2023 RotorPy Authors).
- Spencer Folk, James Paulos, Vijay Kumar, “RotorPy: A Python-based Multirotor Simulator with Aerodynamics for Education and Research”, ICRA 2023 UAV simulator workshop contribution: https://arxiv.org/abs/2306.04485.
- RotorPy는 외부 의존성으로 사용하며 수정된 소스 사본을 포함하지 않습니다. 복제/배포할 때 원저작권·라이선스 고지를 보존하세요.
