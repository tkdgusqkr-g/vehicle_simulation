# 빌드와 실행

M1 기준 안내입니다. 설계는 [design.md](design.md)를 보세요.

## M1에서 되는 것

- **로지텍 G29**(휠 + 페달)로 운전, **포스 피드백(FFB)** 포함. 휠이 없으면 키보드
- L2 차량 모델: 네 바퀴 각각의 타이어(과도 슬립), 하중 이동, 바퀴 회전(휠스핀·락업)
- 구동계: 4WD 인휠, 단일 모터 + 디퍼렌셜(open / LSD / locked) 등 YAML로 구성
- 모터(토크·출력 한계, 손실, 온도 디레이팅)와 배터리(OCV-R-RC 등가회로, 온도, BMS 전류 한계)
- 참조 VCU: 토크 분배, 회생 제동, 휠별 슬립 제어(트랙션 컨트롤), 출력 제한(설정값과 BMS 중 작은 쪽)
- 타이어: 기본 Magic Formula 모델 또는 `.tir` 파일(MF 5.2/6.1)
- 헤드리스 실행, Python 바인딩, MCAP 로그, 비트 단위 결정론 (M0부터)

---

## Windows

### 준비물

| 항목 | 비고 |
|---|---|
| Visual Studio 2022 | 설치할 때 **"C++를 사용한 데스크톱 개발"** 워크로드 선택 (MSVC와 CMake 포함) |
| Python 3.10 이상 | python.org 설치본, 설치할 때 "Add python.exe to PATH" 선택 |
| Git | |
| 인터넷 | 처음 빌드할 때 라이브러리(yaml-cpp, SDL3, IXWebSocket 등)를 자동으로 내려받음 |
| Logitech G HUB | G29를 쓸 때. 로지텍 홈페이지에서 설치 |

### 빌드

시작 메뉴의 **"Developer PowerShell for VS 2022"**에서:

```powershell
git clone https://github.com/tkdgusqkr-g/vehicle_simulation.git
cd vehicle_simulation
python -m pip install pybind11 numpy pytest mcap
cmake -S . -B build
cmake --build build --config Release
```

처음 `cmake -S . -B build`는 라이브러리를 내려받느라 1~2분, 첫 빌드는 SDL3 때문에 몇 분 더 걸립니다.

### 테스트

```powershell
ctest --test-dir build -C Release --output-on-failure
python -m pytest python/tests
```

---

## 로지텍 G29로 운전하기

### 1. 휠 준비 (처음 한 번)

1. G29를 USB에 연결하고 G HUB를 켭니다. G HUB에 휠이 보이면 됩니다.
2. G29 본체의 모드 스위치는 PC에서 쓸 때 로지텍 안내대로 둡니다 (보통 PS3 위치).
3. G HUB에서 **회전 범위(Operating Range)**를 정합니다.
   - 900° (기본값): 실제 차보다 많이 돌려야 합니다. 차량 조향 한계(예제 차량 ±110°)를 넘으면 FFB가 밀어냅니다.
   - **220° (권장)**: 휠의 물리 한계가 차량 조향 한계와 같아집니다. 이때는 프로필의 `steer.range_deg`도 220으로 바꿉니다 (아래 3단계).
4. 장치가 보이는지 확인합니다.

```powershell
build\core\Release\vsim-input.exe list
```

`[0] Logitech G29 Driving Force Racing Wheel USB  usb 046d:c24f  axes 4 ... force feedback (constant)`처럼 나오면 됩니다.

### 2. 보정 (권장)

기본 프로필 [`config/input/logitech_g29.yaml`](../config/input/logitech_g29.yaml)은 일반적인 축 배치를 가정합니다. 실제 축 번호, 페달 방향, FFB 방향은 PC 설정에 따라 다를 수 있어서 보정 도구로 측정해 두는 것이 안전합니다.

```powershell
build\core\Release\vsim-input.exe calibrate
```

화면 안내에 따라 휠 중앙 → 왼쪽 90° → 가속 페달 → 브레이크 → 클러치를 차례로 하고, 마지막에 휠에서 손을 떼면 FFB 방향을 잽니다 (휠이 잠깐 저절로 돕니다). 결과는 `config/input/logitech_g29_....yaml`로 저장됩니다. 시뮬레이터는 이 폴더의 프로필 중 장치에 맞는 것을 쓰는데, 여럿이 맞으면 `priority`가 높은 것을 고릅니다. 보정한 프로필은 `priority: 10`이라 기본 프로필(0)보다 우선합니다.

확인용 명령:

| 명령 | 내용 |
|---|---|
| `vsim-input monitor` | 조향각, 페달, 버튼 번호를 실시간 표시 |
| `vsim-input ffb-test` | 휠을 왼쪽 → 오른쪽으로 밀어 봄 (`--level 0.4`로 세기) |

### 3. 주행

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml
```

그다음 `ui\live\index.html`을 Chrome이나 Edge로 엽니다. 콘솔에 `Steering wheel: Logitech G29 ... (force feedback on)`이 나오고, 라이브 뷰 왼쪽 위 "입력"에 휠 이름이 보이면 휠로 운전 중입니다. 휠을 뽑으면 자동으로 키보드로 넘어가고, 다시 꽂으면 휠로 돌아옵니다.

- FFB는 타이어의 셀프 얼라이닝 토크와 캐스터 트레일로 계산한 핸들 반력입니다. 그립 한계에 가까워지면 핸들이 가벼워집니다.
- 일시정지(Space)하면 노면 반력이 0.3초에 걸쳐 사라지고, 휠 자체의 감쇠와 조향 한계 스토퍼만 남습니다.
- 브라우저 창에 포커스가 없어도 휠 입력과 FFB는 계속 동작합니다.

### FFB 조절

프로필 YAML의 `ffb:` 아래 값을 바꾸고 vsim-core를 다시 시작합니다.

| 키 | 뜻 | 기본값 |
|---|---|---|
| `gain` | 차량 핸들 토크(N·m) → 휠 토크 비율. 무거우면 줄이고 가벼우면 키움 | 0.15 |
| `max_torque_nm` | 휠의 최대 토크 (G29 약 2.2) | 2.2 |
| `damping_nm_s_per_rad` | 휠 감쇠 (떨림 억제) | 0.03 |
| `friction_nm` | 마찰감 | 0.05 |
| `end_stop_nm_per_rad` | 조향 한계를 넘었을 때 밀어내는 강성 | 8 |
| `invert` | FFB 방향 반전 (calibrate가 정함) | true |

라이브 뷰의 "FFB" 칸에 휠 토크가 나오고, "(포화)"가 자주 보이면 `gain`을 줄이세요. 예제 차량은 1 g 선회에서 핸들 반력이 약 12 N·m이고, 기본 `gain` 0.15면 휠에는 약 1.8 N·m가 걸립니다.

### 키보드로 주행

휠이 없으면 `drive.yaml`이 그대로 키보드로 동작합니다. 휠을 연결한 채 키보드만 쓰려면 `--no-wheel`을 붙이거나 `--set driver.input=keyboard`를 씁니다. 예전 `keyboard.yaml` 시나리오도 그대로 있습니다.

| 키 | 동작 |
|---|---|
| ↑ / W | 가속 |
| ↓ / S | 브레이크 |
| ← → / A D | 조향 (속도가 높을수록 최대 조향각이 줄어듦) |
| Space | 일시정지 |
| R | 처음부터 다시 |
| C | 카메라 (진행 방향 위 ↔ 북쪽 위) |
| + − / 마우스 휠 | 확대·축소 |

휠 버튼으로 리셋·일시정지를 하려면 `vsim-input monitor`로 버튼 번호를 보고 프로필의 `buttons.reset`, `buttons.pause`에 적습니다. 종료는 콘솔에서 Ctrl+C입니다.

---

## 헤드리스 실행과 로그

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/sine_steer.yaml --log-dir runs
```

`runs\<날짜-시각>_sine_steer\` 아래에 아래 파일이 생깁니다.

| 파일 | 내용 |
|---|---|
| `log.mcap` | 시계열 로그 (100 Hz). PlotJuggler나 Foxglove로 열기 |
| `scenario.resolved.yaml` | 오버라이드까지 모두 적용된 최종 파라미터 |
| `meta.json` | 시드, 시뮬레이터 버전·커밋, realism 설정, 신호 단위 |
| `summary.json` | 주행 시간, 거리, 최고 속도 |

직접 운전하는 시나리오에도 `--log-dir runs`를 붙이면 주행 기록이 남습니다.

### 주요 신호

| 신호 | 내용 |
|---|---|
| `veh.chassis.*` | 위치, 속도, 가속도, yaw rate, 차체 슬립각 |
| `veh.wheel.<fl/fr/rl/rr>.*` | 수직 하중, 타이어 힘·모멘트, 슬립률·슬립각, 바퀴 회전속도, 조향각 |
| `veh.ffb.torque_sw` | 노면이 핸들에 주는 토크 (N·m, 반시계 +) |
| `pt.motor.<id>.*` | 토크 명령·실제, 회전속도, 가용 토크, 권선 온도, 손실, 전력 |
| `bat.*` | SoC, 단자 전압·전류·전력, OCV, 온도, BMS 한계 (`bat.limit.*`) |
| `vcu.*` | 출력 한계, 회생 비율, 바퀴별 슬립과 슬립 제어 계수 |
| `energy.*` | 에너지 흐름 누적값 (배터리 화학 에너지, 각 손실, 운동에너지, `residual`은 수지 오차) |

### 자주 쓰는 옵션

| 옵션 | 설명 |
|---|---|
| `--set key=value` | 파라미터 덮어쓰기 (여러 번 가능). 예: `--set vehicle.mass_kg=300 --set control.power_limit_kw=60` |
| `--mode fast` | 시나리오 설정과 상관없이 헤드리스로 최대 속도 실행 |
| `--duration 20` | 20초(시뮬레이션 시간)만 실행 |
| `--port 9000` | 라이브 뷰 포트 변경. 브라우저 주소에 `?ws=ws://127.0.0.1:9000`을 붙임 |
| `--host 0.0.0.0` | 다른 PC의 브라우저에서 접속 허용 |
| `--no-wheel` | 휠을 찾지 않음 |
| `--input-profiles <폴더>` | 휠 프로필 폴더 지정 (기본: 소스 트리의 `config/input`) |

---

## Python

`python` 폴더를 경로에 추가하고 씁니다.

```python
import sys
sys.path.insert(0, "python")
import vsim

# 한 번에 실행하고 기록 받기
rec = vsim.run_scenario("templates/projects/example_fs_ev/scenarios/sine_steer.yaml")
print(rec["t"][-1], rec["veh.chassis.vx"].max(), rec["bat.soc"][-1])

# 직접 조종하기 (driver.type을 external로)
sim = vsim.Simulation(
    "templates/projects/example_fs_ev/scenarios/straight_accel.yaml",
    overrides={"driver.type": "external", "vehicle.mass_kg": 300},
)
sim.set_driver_command(throttle=1.0, steer=0.1)  # 핸들각 [rad], 페달 [0, 1]
sim.run(3.0)
print(sim.get("veh.chassis.vx"), sim.get("veh.wheel.rl.slip_ratio"))
```

---

## 시나리오와 차량 프로젝트

```text
templates/projects/example_fs_ev/      예제 차량 프로젝트 (대표값, 실차 아님)
├─ project.yaml                        이름, 스키마 버전, 하위 파일 경로
├─ vehicle/vehicle.yaml                차량 파라미터 (모델, 서스펜션, 타이어, 구동계, 모터, 배터리, 브레이크)
├─ vehicle/tires/example_fs_10in.tir   예제 Magic Formula 타이어 파일 (합성 데이터)
├─ control/vcu.yaml                    참조 VCU 설정 (토크 분배, 회생, 슬립 제어, 출력 한계)
└─ scenarios/                          drive / keyboard / sine_steer / straight_accel
```

자기 차량은 이 폴더를 복사해서 값을 바꾸면 됩니다. 시나리오의 `project:`는 시나리오 파일 기준 상대 경로입니다.

| 바꾸고 싶은 것 | 위치 |
|---|---|
| 차량 모델 | `vehicle.yaml`의 `model: two_track` (L2) 또는 `single_track` (L1, 빠른 검토용) |
| 구동계 토폴로지 | `vehicle.yaml`의 `driveline:` 목록 (주석에 후륜 단일 모터 + LSD 예시) |
| 타이어를 `.tir`로 | `tire: {model: mf, file: vehicle/tires/<파일>.tir}` (축별로는 `tire.front.file`) |
| 드라이버 | 시나리오의 `driver.type`: `dil`(휠/키보드), `keyboard`, `maneuver`(스크립트), `external`(Python) |

---

## Linux

```bash
python3 -m pip install pybind11 numpy pytest mcap
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
python3 -m pytest python/tests
./build/core/vsim-core --scenario templates/projects/example_fs_ev/scenarios/drive.yaml
```

Linux에서 휠을 쓰려면 `/dev/input/event*`, `/dev/hidraw*` 읽기 권한이 필요합니다 (보통 `input` 그룹).

---

## 문제 해결

| 증상 | 해결 |
|---|---|
| `vsim-input list`에 휠이 안 보임 | G HUB에서 휠이 보이는지 확인. USB를 다시 꽂고 G HUB 재시작 |
| 핸들을 돌리면 차가 반대로 감 | `vsim-input calibrate`를 다시 하거나 프로필의 `steer.invert`를 바꿈 |
| 페달을 안 밟아도 가속/브레이크가 들어감 | 페달 방향이 반대. `calibrate`를 다시 하거나 프로필의 `released`/`pressed`를 맞바꿈 |
| 가속과 브레이크가 한 축으로 합쳐짐 | G HUB에서 "결합된 페달(Combined Pedals)" 끄기 |
| FFB가 반대로 당김 (핸들이 바깥으로 감김) | 프로필의 `ffb.invert`를 바꾸거나 `calibrate`. 휠이 조향 한계보다 90° 넘게 0.5초 이상 나가 있으면 FFB가 자동으로 멈추고 콘솔과 라이브 뷰에 안내가 뜹니다 (한계 안으로 돌아오면 재개) |
| FFB가 너무 세거나 약함 | 프로필의 `ffb.gain` 조절. G HUB의 FFB 강도도 확인 |
| 조향 각도가 화면과 실제가 다름 | 프로필 `steer.range_deg`를 G HUB 회전 범위와 같게 |
| `Could not find a package configuration file provided by "pybind11"` | `python -m pip install pybind11`. Python 바인딩이 필요 없으면 `-DVSIM_BUILD_PYTHON=OFF` |
| SDL3 다운로드·빌드 실패 | 네트워크 확인. 휠 없이 쓰려면 `-DVSIM_WITH_SDL=OFF` (키보드만) |
| CMake가 다른 Python을 잡음 | `-DPython_EXECUTABLE="C:\경로\python.exe"` |
| `cannot listen on 127.0.0.1:8765` | 다른 프로그램이 포트 사용 중. `--port`로 변경 |
| 라이브 뷰가 "연결 끊김" | vsim-core가 realtime 모드로 실행 중인지 확인 (fast 모드는 서버를 열지 않음) |
| `import vsim` 실패 | 빌드 후 `python/vsim/` 안에 `_vsim...pyd`(Windows) 또는 `.so`(Linux)가 있는지 확인 |
