# 빌드와 실행

M0 기준 안내입니다. 설계는 [design.md](design.md)를 보세요.

## M0에서 되는 것

- 키보드로 평지에서 차를 몰기 (브라우저 라이브 뷰)
- 같은 시나리오를 헤드리스로 빠르게 돌리기 (실시간의 수백 배)
- Python에서 시뮬레이션 실행하고 결과를 numpy 배열로 받기
- 실행 결과를 MCAP 로그와 메타데이터로 저장 (PlotJuggler, Foxglove에서 열림)
- 같은 시나리오 + 같은 시드 → 비트 단위로 같은 결과

차량 모델은 L1(single-track)입니다. 바퀴 회전 상태, 노면, 센서, CAN은 다음 마일스톤부터 들어갑니다 ([design.md §19](design.md)).

---

## Windows

### 준비물

| 항목 | 비고 |
|---|---|
| Visual Studio 2022 | 설치할 때 **"C++를 사용한 데스크톱 개발"** 워크로드 선택 (MSVC와 CMake 포함) |
| Python 3.10 이상 | python.org 설치본, 설치할 때 "Add python.exe to PATH" 선택 |
| Git | |
| 인터넷 | 처음 빌드할 때 라이브러리(yaml-cpp, IXWebSocket 등)를 자동으로 내려받음 |

### 빌드

시작 메뉴의 **"Developer PowerShell for VS 2022"**에서:

```powershell
git clone https://github.com/tkdgusqkr-g/vehicle_simulation.git
cd vehicle_simulation
python -m pip install pybind11 numpy pytest mcap
cmake -S . -B build
cmake --build build --config Release
```

처음 `cmake -S . -B build`는 라이브러리를 내려받느라 1~2분 걸립니다.

### 테스트

```powershell
ctest --test-dir build -C Release --output-on-failure
python -m pytest python/tests
```

### 키보드로 주행

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/keyboard.yaml
```

그다음 `ui\live\index.html`을 더블클릭해서 Chrome이나 Edge로 엽니다. 왼쪽 위에 "연결됨"이 뜨면 됩니다.

| 키 | 동작 |
|---|---|
| ↑ / W | 가속 |
| ↓ / S | 브레이크 |
| ← → / A D | 조향 (속도가 높을수록 최대 조향각이 줄어듦) |
| Space | 일시정지 |
| R | 처음부터 다시 |
| C | 카메라 (진행 방향 위 ↔ 북쪽 위) |
| + − / 마우스 휠 | 확대·축소 |

종료는 콘솔에서 Ctrl+C입니다.

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

키보드 시나리오에도 `--log-dir runs`를 붙이면 직접 운전한 기록이 남습니다.

### 자주 쓰는 옵션

| 옵션 | 설명 |
|---|---|
| `--set key=value` | 파라미터 덮어쓰기 (여러 번 가능). 예: `--set vehicle.mass_kg=300 --set control.power_limit_kw=60` |
| `--mode fast` | 시나리오 설정과 상관없이 헤드리스로 최대 속도 실행 |
| `--duration 20` | 20초(시뮬레이션 시간)만 실행 |
| `--port 9000` | 라이브 뷰 포트 변경. 브라우저 주소에 `?ws=ws://127.0.0.1:9000`을 붙임 |
| `--host 0.0.0.0` | 다른 PC의 브라우저에서 접속 허용 |

---

## Python

`python` 폴더를 경로에 추가하고 씁니다.

```python
import sys
sys.path.insert(0, "python")
import vsim

# 한 번에 실행하고 기록 받기
rec = vsim.run_scenario("templates/projects/example_fs_ev/scenarios/sine_steer.yaml")
print(rec["t"][-1], rec["veh.chassis.vx"].max())

# 직접 조종하기 (driver.type을 external로)
sim = vsim.Simulation(
    "templates/projects/example_fs_ev/scenarios/straight_accel.yaml",
    overrides={"driver.type": "external", "vehicle.mass_kg": 300},
)
sim.set_driver_command(throttle=1.0, steer=0.1)  # 핸들각 [rad], 페달 [0, 1]
sim.run(3.0)
print(sim.get("veh.chassis.vx"), sim.signals()[:5])
```

---

## 시나리오와 차량 프로젝트

```text
templates/projects/example_fs_ev/      예제 차량 프로젝트 (대표값, 실차 아님)
├─ project.yaml                        이름, 스키마 버전, 하위 파일 경로
├─ vehicle/vehicle.yaml                차량 파라미터
├─ control/vcu.yaml                    참조 VCU 설정
└─ scenarios/                          keyboard / sine_steer / straight_accel
```

자기 차량은 이 폴더를 복사해서 값을 바꾸면 됩니다. 시나리오의 `project:`는 시나리오 파일 기준 상대 경로입니다.

---

## Linux

```bash
python3 -m pip install pybind11 numpy pytest mcap
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
python3 -m pytest python/tests
./build/core/vsim-core --scenario templates/projects/example_fs_ev/scenarios/keyboard.yaml
```

---

## 문제 해결

| 증상 | 해결 |
|---|---|
| `Could not find a package configuration file provided by "pybind11"` | `python -m pip install pybind11`. Python 바인딩이 필요 없으면 `-DVSIM_BUILD_PYTHON=OFF` |
| CMake가 다른 Python을 잡음 | `-DPython_EXECUTABLE="C:\경로\python.exe"` |
| `cannot listen on 127.0.0.1:8765` | 다른 프로그램이 포트 사용 중. `--port`로 변경 |
| 라이브 뷰가 "연결 끊김" | vsim-core가 realtime 모드로 실행 중인지 확인 (fast 모드는 서버를 열지 않음) |
| `import vsim` 실패 | 빌드 후 `python/vsim/` 안에 `_vsim...pyd`(Windows) 또는 `.so`(Linux)가 있는지 확인 |
