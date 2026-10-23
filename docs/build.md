# 빌드와 실행

M9 기준 안내입니다. 설계는 [design.md](design.md)를 보세요.

## 지금 되는 것

**M11 (디테일 마감, 진행 중)**
- **타이어 온도·공기압** (PH-13): 바퀴마다 트레드 표면과 카커스(내부 공기) 온도. 미끄럼 마찰열·구름 저항 열·브레이크 림 열로 데워지고 공기·노면으로 식음. 표면 온도의 최적 창(포물선)과 공기압(이상기체로 내부 온도를 따라감)이 그립을 바꿈. 차가운 타이어로 나가면 첫 랩이 1초 이상 느림
- **브레이크 열** (PH-17): 디스크 온도(브레이크 일률로 가열, 속도에 따른 대류·복사 냉각)와 패드 마찰 곡선(차가우면 덜 듣고 뜨거우면 페이드). 앞뒤 온도가 달라 제동 밸런스가 바뀜
- **바람** (PH-15): 정상풍 + 1-cos 돌풍 + 난류(차가 지나가는 얼어붙은 소용돌이, 시드 고정). 공력을 공기에 대한 속도로 계산해서 측풍에서 측력과 yaw 모멘트가 생기고, 바람이 한 일까지 에너지 수지에 들어감
- **셀 편차** (PH-06): 직렬 셀 그룹마다 용량·내부저항·초기 SoC가 다름 (`realism.variation: real`에서 시드로 표본, 약한 그룹 직접 지정 가능). 모든 그룹이 같은 전류를 받으므로 그룹 SoC·RC 전압은 평균 그룹에서 정확히 계산되고, BMS 한계는 가장 약한 그룹이 정함 → 낮은 SoC에서 출력 제한이 먼저 걸림
- **구동계 비틀림·백래시, 토크 리플** (PH-04): 감속기와 바퀴 사이 비틀림 강성·감쇠와 유격(로터가 자기 좌표를 가짐), 로터 각도의 고조파 토크 리플과 코깅. 출발 때 축 토크가 유격을 지나며 역전·튀고, 토크를 끊으면 덜컹거리며, 슬립 제어가 이 진동을 바퀴 슬립으로 오인
- MPC 드라이버가 타이어 그립 변화를 느끼고 기대 그립·목표 속도를 낮춤
- 라이브 뷰 대시보드에 바퀴별 타이어 온도(색)·공기압·브레이크 온도

**M10 (스스로 한계까지)**
- **MPC 드라이버** (`driver.type: mpc`): 1.5~2초 앞을 예측하면서 조향과 가감속을 매 50 ms 최적화합니다. 차량의 타이어 데이터로 자기 모델(축별 타이어 곡선, 하중 이동)을 맞추고, 달리면서 앞뒤 축이 실제로 얼마나 무는지 배웁니다
- **추종 모드** (`mode: track`): 주행 라인과 속도 프로파일을 사람처럼 따라감. 반응 지연 0.15 s, 근육 지연(2차 필터), 핸들 속도 한계, 손떨림. 예제 오토크로스 20.3 s
- **한계 주행 모드** (`mode: race`, MPCC): 트랙 경계와 마찰 한계 안에서 진행 거리를 최대화. 예제 오토크로스 19.1 s (Pure Pursuit 20.8 s)
- 두 모드 모두 고저차·뱅크·연석·젖은 구간·노면 요철과 모든 비이상성(센서·액추에이터·CAN·전기·개체 편차)을 켠 서킷에서 코스 이탈 없이 완주
- **속도 프로파일**: `python -m vsim.profiles`로 주행 기록(사람·레이싱 MPC·실차 로그)에서 한 랩의 속도를 뽑아 추종 모드에 줌
- 라이브 뷰에 MPC가 계획한 경로(점선)

**M9 (전기가 흐른다)**
- **전기 네트워크**: 프로젝트의 `electrical/netlist.yaml`로 고전압(축전지·퓨즈·AIR·프리차지·방전·HV 케이블·DC 링크·인버터), 셧다운 회로, 저전압(LV 배터리·DC-DC·퓨즈·ECU·펌프·팬), 센서 배선(5 V 레귤레이터·APPS 포텐셔미터·커넥터 핀·ADC)을 정하면, 매 1 ms 수정 절점 해석(MNA)으로 풉니다
- 켜면 배터리 ↔ 인버터가 직결이 아닙니다: AIR가 붙어야 모터에 전기가 가고, DC 링크 전압으로 모터 출력이 정해지고, 배터리 전류는 회로에서 나옵니다
- **프리차지·셧다운**: 셧다운 회로가 닫히면 AIR−·프리차지 릴레이 → DC 링크 95 % → AIR+ 순서. 규정(90 %) 위반·실패(타임아웃)·컨택터 채터링 감지 (PH-16)
- **ECU 전원**: 최소 전압 아래로 떨어지면 리셋, 부팅 시간 뒤 재시작. 꺼진 동안 그 ECU의 CAN 노드가 멈추고 출력은 0 (PH-07)
- **센서 배선**: APPS 신호가 배선·커넥터 핀·접지를 거쳐 VCU ADC로. 접촉 저항·단선이 APPS 타당성 검사로 이어짐 (PH-10)
- **고장 주입**: 단선, 단락, 접촉 저항(커넥터 핀), 퓨즈 단선, 릴레이 고착, 간헐 접촉
- 전선 저항·온도 상승·허용 전류, 전압 강하, LV 전력 예산, DC-DC 여유 → `summary.json`의 `electrical`
- **회로 편집기** (`ui/elec/index.html`): 회로도에서 부품을 끌고 단자를 망에 연결, 속성 편집, 동작점 검사, 실행 중 전압·전류·접점 상태, 고장·하네스·LV 전력 예산·이벤트 탭

**M8 (CAN으로 말한다)**
- **가상 CAN**: 프로젝트의 DBC와 `can/network.yaml`로 버스·노드·신호 연결을 정하면, VCU의 토크 명령은 CAN 프레임으로 인버터(플랜트)에, 모터 속도·BMS 한계·IMU 값은 CAN 프레임으로 VCU에 갑니다
- 충실도 3단계: `ideal`(메시지 주기만), `timing`(비트 스터핑까지 센 프레임 길이, ID 중재, 대기열, 노드마다 클록 오차·지터), `faults`(+ 프레임 손실·지연·에러 프레임·bus-off·송신 누락)
- 수신 타임아웃(그때 마지막 값 유지 또는 0), 수신 태스크마다 프레임 놓침·중복 집계, 버스 부하·메시지 지연 신호와 실행 요약
- **버스 로그**: 실행 폴더에 Vector ASC(`<버스>.asc`), candump(`<버스>.log`) → SavvyCAN·CANalyzer로 열기
- **CAN 네트워크 편집기** (`ui/can/index.html`): 버스·DBC·노드·신호 연결·타임아웃·추가 트래픽·고장 편집, DBC 올리기·내보내기, 예상 부하와 실제 부하, 메시지별 지연·손실·타임아웃
- 예제 DBC(`can/fs_ev.dbc`, VCU·인버터 4개·BMS·IMU)

**M7 (실차 코드로 달린다)**
- **플러그인**: 팀 코드를 VCU·드라이버 자리에 넣거나 VCU 다음에 블록으로 더합니다. C/C++ 공유 라이브러리(`sdk/c/include/vsim_plugin.h`), **FMU**(FMI 2.0 Co-Simulation: Simulink·Modelica·AMESim 등에서 내보낸 것), **Python** 클래스. 프로젝트의 `plugins/components.yaml`에서 포트를 신호에 연결하고, 단위는 자동 변환
- **Simulink `vsim_export`**: 모델 하나를 명령 하나로 플러그인으로. Embedded Coder가 있으면 실차와 같은 C 코드로, 없으면 FMU로. 포트 이름으로 신호에 자동 연결, 모델 버전·해시 기록
- **플러그인 페이지** (`ui/plugins/index.html`): 파일 올리기, 포트 불러오기, 신호 연결, 켜고 끄기, 저장하면 바로 다시 불러오기(잘못된 연결은 이전 설정으로 되돌림). 실행 시간 표시
- 예제: 같은 토크 벡터링을 C++·FMU·Python으로 만든 세 플러그인(결과가 비트 단위로 같음), C로 쓴 최소 VCU
- 실행 기록: 플러그인 파일 SHA-256·버전·연결(`meta.json`), 호출당 실행 시간(`summary.json`)

**M6 (센서로 본다)**
- **센서 신호 체인**: VCU가 참값 대신 센서가 잰 값을 받습니다. IMU(장착 위치의 구심·접선 가속도, 차체 롤·피치와 노면 경사의 중력 성분, 장착 각도), 휠속(톱니 펄스 시간 → 저속에서 끊김), APPS 2채널, 브레이크 압력, 조향각, GNSS(지연·끊김), 전류·전압·온도, 서스펜션 변위
- 채널마다 개체 편차(스케일·바이어스), 잡음(데이터시트의 noise density, bias instability, random walk), 대역폭, 지연, 양자화, 포화. 모터 회전 차수 진동과 **aliasing**(anti-alias 필터 유무)
- **고장 주입**: 단선·단락·고착·오프셋·잡음을 시각과 채널을 정해서
- **VCU**: FSAE APPS 타당성 검사(10 %·100 ms → 토크 차단), 브레이크·APPS 동시 작동 차단(BSPC), 휠속 + IMU 차속 추정으로 트랙션 컨트롤
- **현상 탐지**: `python -m vsim.phenomena <시나리오>` → 토크 차단, TC 개입, 휠속·GNSS 끊김, 진동 피크, 분류별 원인 분리 표
- 차량 빌더에 가속 페달 센서, 센서별 설정 보존. 규정 점검에 APPS·BSE

**M5 (노면 위를 구른다)**
- **L3 차량 모델** (`vehicle.model: full`, 예제 차의 기본값): 차체가 상하·롤·피치로 움직이고, 네 바퀴의 비현가 질량이 타이어 스프링 위에서 노면을 따라갑니다. 스프링·디그레시브 댐퍼·범프/처짐 스톱·안티롤바. 에너지 수지가 그대로 맞습니다 (잔차 약 1e-7)
- **하드포인트 기구학(K&C)**: 더블 위시본 + 푸시로드(선택: 로커)를 풀어서 캠버·범프 스티어·접지점 이동(잭킹, 롤센터)·모션비·조향축 모멘트를 계산. `vsim-core --export-kc`로 곡선과 킹핀각·캐스터·스크럽·트레일·애커만 확인
- **FFB가 노면을 느낌**: 핸들 토크를 조향축에 대한 타이어 힘의 모멘트로 계산해서, 요철·연석·하중 변화가 핸들로 전달됨
- **3D 노면**: 트랙 고도(스플라인)·뱅크·연석(럼블)·ISO 8608 거칠기·마찰 구간(젖은 노면 등)·잔디(트랙 밖, 미끄럽고 거침). 3D 화면에 고저·연석·지형까지 그려짐
- 타이어: 캠버(MF 5.2/6.1 캠버 항, 단순 모델의 캠버 추력), 노면 마찰 배율
- 새 템플릿 `hill_circuit` (고저차 5 m, 뱅크 코너, 연석, 젖은 구간), 트랙 편집기에 노면·연석·마찰 구간·고도 단면
- 4포스트 리그 입력(`rig.<바퀴>.z`)과 검증: 1/4차 주파수 응답(V-09), 노면 PSD(V-10)

**M4 (우리 차로 구른다)**
- **차량 빌더** (`ui/builder/index.html`): STEP·IGES·glTF·STL을 브라우저에서 열고, 부품 역할(차체·바퀴·업라이트·링크·드라이버)과 재질을 정하면 질량·무게중심·관성·비현가 질량·축거·윤거·타이어 반경을 계산. 서스펜션 하드포인트와 센서 위치를 3D에서 찍어서 프로젝트에 저장하면 시뮬레이터가 그 차로 다시 시작
- 예제 차량을 **FSAE 규정 기준**으로 정리 (600 V 이하, 80 kW, 축거 1530 mm, 10" 휠, 세그먼트 120 V·6 MJ 이하). `python -m vsim.rules <프로젝트>`로 규정 점검
- 그래픽: 하늘·환경광·그림자·앰비언트 오클루전·블룸, 아스팔트·잔디 질감, 카본·클리어코트 도장, 주변 환경(나무, 관중석, 피트 텐트, 출발 게이트). 서스펜션 링크와 드라이버까지 있는 포뮬러 차량 모델. Unreal 뷰어도 같은 모델과 재질을 공유 메모리로 받음

**M3 (보면서 구른다)**
- **3D 주행 화면**: 브라우저(`ui/view3d/index.html`, 바로 동작)와 Unreal Engine 5 뷰어(`ui/unreal`, 직접 빌드 필요)
- 트랙 3D 모델 자동 생성: 아스팔트, 경계선, 출발·구간선, FS 콘(왼쪽 파랑, 오른쪽 노랑, 출발 주황), 잔디. `.glb`로 내보내기 가능
- 공유 메모리 링크: 차량 상태를 약 1 ms마다 내보내고, 뷰어의 키와 리셋·일시정지를 받음. `vsim-link`로 점검
- **입력 보정 페이지** (`ui/input/index.html`): 축·페달·버튼·FFB 방향을 보면서 보정하고, 재시작 없이 적용·저장
- 입력에서 화면까지 지연 측정 (목표 50 ms 미만)

**M2 (트랙 위를 구른다)**
- 트랙: 센터라인 점 + 폭 + 출발/결승·구간 + 주행 라인. 템플릿(가속, 스키드패드, 원선회, 슬라럼, 더블 레인 체인지, 오토크로스)
- **트랙 편집기** (`ui/editor/index.html`): 점 추가·이동·삭제, 폭, 구간, 주행 라인(최소 곡률 자동 계산 + 핸들 편집), 검사, 실행 중인 시뮬레이터로 바로 보내기
- 랩 타이밍: 랩·구간 시간, 코스 이탈, 무효 랩. 라이브 뷰에 트랙과 랩 정보 표시
- 자동 주행: Pure Pursuit + 속도 프로파일로 주행 라인을 따라 랩 주행
- 표준 시험용 입력: 사인 스윕, 사인 위드 드웰 (Maneuver)
- **시나리오 테스트 러너**: `python -m vsim.testing` (판정식 → 통과/실패 리포트)

**M1까지**
- **로지텍 G29**(휠 + 페달)로 운전, **포스 피드백(FFB)** 포함. 휠이 없으면 키보드
- L2 차량 모델: 네 바퀴 각각의 타이어(과도 슬립), 하중 이동, 바퀴 회전(휠스핀·락업)
- 구동계: 4WD 인휠, 단일 모터 + 디퍼렌셜(open / LSD / locked) 등 YAML로 구성
- 모터(토크·출력 한계, 손실, 온도 디레이팅)와 배터리(OCV-R-RC 등가회로, 온도, BMS 전류 한계)
- 참조 VCU: 토크 분배, 회생 제동, 휠별 슬립 제어(트랙션 컨트롤), 출력 제한(설정값과 BMS 중 작은 쪽)
- 타이어: 기본 Magic Formula 모델 또는 `.tir` 파일(MF 5.2/6.1)
- 헤드리스 실행, Python 바인딩, MCAP 로그, 비트 단위 결정론

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
| Unreal Engine 5.4 이상 (선택) | Unreal 뷰어를 쓸 때. Visual Studio에 **"C++를 사용한 게임 개발"** 워크로드도 추가 |

### 빌드

시작 메뉴의 **"Developer PowerShell for VS 2022"**에서:

```powershell
git clone https://github.com/tkdgusqkr-g/vehicle_simulation.git
cd vehicle_simulation
python -m pip install pybind11 numpy pytest mcap pyyaml
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
   - **220° (권장)**: 휠의 물리 한계가 차량 조향 한계와 같아집니다. 이때는 보정(2단계)에서 회전 범위를 220으로 입력하거나, 프로필의 `steer.range_deg`를 220으로 바꿉니다.
4. 장치가 보이는지 확인합니다.

```powershell
build\core\Release\vsim-input.exe list
```

`[0] Logitech G29 Driving Force Racing Wheel USB  usb 046d:c24f  axes 4 ... force feedback (constant)`처럼 나오면 됩니다.

### 2. 보정 (권장)

**보정 페이지(권장)**: `drive.yaml`로 vsim-core를 실행한 채 `ui\input\index.html`을 엽니다. 단계를 따라 하면 핸들 축과 방향, 페달 범위, 리셋·일시정지 버튼, FFB 방향을 잡습니다. 단계마다 축이 움직이는 모습이 보이고, 적용 전에 미리보기로 확인할 수 있습니다. **적용하고 저장**을 누르면 `config/input/<이름>.yaml`에 저장되고, 시뮬레이터를 다시 시작하지 않아도 바로 적용됩니다. FFB 세기(배율, 최대 토크, 감쇠, 마찰)도 이 페이지에서 바꿀 수 있습니다.

**콘솔 도구**: 기본 프로필 [`config/input/logitech_g29.yaml`](../config/input/logitech_g29.yaml)은 일반적인 축 배치를 가정합니다. 실제 축 번호, 페달 방향, FFB 방향은 PC 설정에 따라 다를 수 있어서 보정 도구로 측정해 두는 것이 안전합니다.

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

## 3D 화면

모니터 1대 기준입니다. 3D 화면을 전체 화면으로 띄우고, 라이브 뷰와 편집기는 필요할 때만 엽니다.

### 브라우저 3D 화면

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml --set track.template=autocross
```

`ui\view3d\index.html`을 Chrome이나 Edge로 열면 시뮬레이터가 만든 트랙 3D 모델과 차량이 나옵니다. three.js를 인터넷(cdn.jsdelivr.net)에서 받으므로 처음 열 때 인터넷이 필요합니다.

| 키 | 동작 |
|---|---|
| ↑↓←→ / WASD | 운전 (휠이 있으면 휠) |
| C | 카메라: 추적 → 콕핏 → 노즈 → TV(트랙 옆 카메라) → 위 |
| R / Space | 리셋 / 일시정지 |
| H / F | HUD 숨김 / 전체 화면 |

- 왼쪽 위 **키 → 화면**: 가속 키를 누른 순간부터 그 결과가 그려진 프레임까지의 시간입니다. 시뮬레이터, WebSocket, 렌더링이 모두 포함됩니다. 가속 키를 몇 번 눌렀다 떼면 평균이 나옵니다.
- 상태는 초당 60번 오고, 그 사이 프레임은 속도와 yaw rate로 앞으로 외삽해서 고주사율 모니터에서도 부드럽게 움직입니다.
- 편집기에서 트랙을 보내면 3D 화면의 트랙도 바로 바뀝니다.

### Unreal Engine 5 뷰어

[`ui/unreal/README.md`](../ui/unreal/README.md)를 보세요. 요약하면 다음과 같습니다.
1. UE 5.4 이상과 Visual Studio 2022를 설치합니다.
2. `ui/unreal/VsimViewer/VsimViewer.uproject`를 열고 빌드합니다.
3. vsim-core를 실시간 모드로 켠 다음 Play를 누릅니다.

맵이나 에셋 없이 코드만으로 장면을 만듭니다. **이 프로젝트는 Unreal이 없는 환경에서 작성해서 실제 빌드는 아직 해 보지 않았습니다.** 첫 빌드에서 오류가 나면 메시지를 알려 주세요.

vsim-core는 실시간 모드에서 항상 공유 메모리(`vsim_state_v2`)로 상태를 내보냅니다. 끄려면 `--no-link`를 씁니다. 연결 점검에는 `vsim-link`를 씁니다.

| 명령 | 내용 |
|---|---|
| `vsim-link wait` | 시뮬레이터가 내보내는지 확인 |
| `vsim-link monitor` | 상태를 초당 2번 출력 (위치, 속도, 랩, 트랙 세대, 지연) |
| `vsim-link drive throttle --seconds 2` | 뷰어처럼 키를 보내고, 입력이 상태에 반영되기까지의 지연을 잼 |
| `vsim-link reset` / `pause` | 명령 보내기 |

다른 렌더러(Unity, 자체 프로그램)를 붙이려면 `core/include/vsim/link/vsim_link.h` 하나만 가져가면 됩니다. 표준 C++만 쓰는 헤더입니다.

### 지연 예산 (목표: 입력 → 화면 50 ms 미만)

| 구간 | 크기 |
|---|---|
| 휠 읽기 (SDL, 1 kHz) | ≤ 1 ms |
| 시뮬레이터: 입력 → 상태 공개 | 평균 0.6 ms, 최대 4.5 ms (`vsim-link drive`, 이 개발 환경) |
| Unreal: 상태 → 프레임 | 프레임 시간 1–2개 (120 fps면 8–17 ms) |
| 모니터 | vsync 끄면 0–1 프레임, 60 Hz vsync는 +17–50 ms |

Unreal 프로젝트 설정은 vsync를 끄고 프레임 지연을 줄여 둡니다(`r.VSync=0`, `r.OneFrameThreadLag=0`, `r.GTSyncType=1`). 브라우저 화면은 브라우저가 vsync를 강제하므로 Unreal보다 1–2 프레임 깁니다.

### 트랙 3D 모델 내보내기

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_auto.yaml --export-track-glb autocross.glb
```

```python
import vsim
open("skidpad.glb", "wb").write(vsim.Track("templates/tracks/skidpad.track.json").to_glb())
```

Blender, Unreal(glTF 가져오기), three.js에서 열 수 있습니다. 좌표는 glTF 규약(y가 위쪽)이고, 단위는 m입니다.

---

## 차량 빌더 (내 차 올리기)

CAD로 만든 차를 시뮬레이터에 넣는 페이지입니다. 설계는 [design.md](design.md) §5를 보세요.

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml
```

`ui\builder\index.html`을 Chrome이나 Edge로 엽니다. 시뮬레이터가 꺼져 있어도 계산과 "파일로 받기"는 됩니다. STEP 읽기 모듈(occt-import-js, 약 8 MB)을 처음 한 번 인터넷에서 받습니다.

| 탭 | 하는 일 |
|---|---|
| 1 모델 | STEP(.step .stp), IGES, BREP, glTF(.glb), STL을 끌어다 놓기. 단위(STEP은 보통 mm)와 앞쪽·위쪽 축 지정. "지금 차량의 3D 모델 불러오기"는 하드포인트·센서만 고칠 때 |
| 2 부품 | 부품마다 역할(차체, 드라이버, 바퀴, 업라이트·허브, 서스펜션 링크, 제외)과 재질(밀도). 이름으로 먼저 추정합니다(`wheel`, `tyre`, `upright`, `wishbone`, `driver`, `휠` 등). 저울로 잰 질량이 있으면 부품별로 넣음 |
| 3 질량 | 전체 질량, 무게중심 높이와 앞뒤 위치, 축거, 윤거, 요·롤·피치 관성, 코너별 비현가 질량, 타이어 반경·폭, 바퀴 회전 관성. CAD에 드라이버가 없으면 질량과 위치를 넣어서 추가 |
| 4 서스펜션 | 더블 위시본 + 푸시로드 하드포인트 11개를 앞·뒤 각각 화면에서 클릭(꼭짓점에 붙이기)하거나 숫자로 입력. 오른쪽을 찍어도 왼쪽 기준으로 저장 |
| 5 센서 | IMU, 휠 속도, 조향각, GNSS 등을 추가하고 장착 위치를 클릭. 방향은 (롤, 피치, 요) 도 |
| 6 저장 | 저장할 항목 선택, FSAE 형상 점검, `vehicle/builder.yaml` 미리보기. **프로젝트에 저장하고 다시 불러오기**를 누르면 시뮬레이터가 파일을 쓰고 새 차량으로 다시 시작 |

- 바퀴 4개의 역할을 "바퀴"로 정해야 축거와 원점(무게중심 아래 지면)이 정해집니다. 앞뒤·좌우는 위치로 정합니다.
- 질량은 닫힌 솔리드면 부피 × 밀도, 열린 면(시트 바디)이면 넓이 × 2 mm × 밀도로 계산합니다. 바퀴처럼 CAD가 속이 꽉 찬 원기둥이면 실제보다 무거우니 측정 질량을 넣으세요 (25 kg 넘는 비현가 부품은 경고).
- 서스펜션 링크 질량은 절반씩 차체와 비현가로 나눕니다.
- 저장된 `builder.yaml`은 `vehicle.yaml`의 같은 항목만 덮어씁니다. 지우면 `vehicle.yaml` 값으로 돌아갑니다. 목록(센서, 하드포인트 좌표)은 통째로 바뀝니다.
- 3D 화면(브라우저, Unreal)은 저장한 `car.glb`를 씁니다. 차체는 고정, 바퀴는 회전·조향, 업라이트는 조향만 따라갑니다.

CAD 파일의 질량 특성만 따로 확인하려면:

```powershell
$env:PYTHONPATH = "python"
python -m vsim.massprops chassis.stl --density 1600 --units mm   # 닫힌 STL/glb의 질량·무게중심·관성
python -m vsim.rules templates/projects/example_fs_ev              # FSAE 규정 점검 (builder.yaml 반영)
```

---

## L3 차량 모델과 3D 노면

예제 차량은 `vehicle.yaml`의 `model: full`(L3, 14자유도)로 돕니다. 빠른 배치 계산에는 `--set vehicle.model=two_track`(L2)를 씁니다. L3는 평평한 트랙에서 실시간의 약 30배, 거친 고저 트랙에서 약 7배로 돕니다 (이 개발 환경 기준).

```powershell
# 고저·뱅크·연석·젖은 구간이 있는 서킷을 자동 주행으로 (ui\view3d\index.html로 보기)
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_auto.yaml --set track.template=hill_circuit --mode realtime
# 서스펜션 기구학 확인
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml --export-kc kc.csv
```

| 바꾸고 싶은 것 | `vehicle.yaml` |
|---|---|
| 현가·비현가 질량 | `unsprung_mass_kg: {fl, fr, rl, rr}` (나머지가 차체), `roll_inertia_kgm2`, `pitch_inertia_kgm2` (차체만, 그 무게중심 기준) |
| 타이어 수직 | `tire.vertical_stiffness_n_per_mm`, `vertical_damping_n_s_per_m`, `contact_length_m` (노면을 이 길이로 평균), `wheel.unloaded_radius_m` |
| 서스펜션 기하 | `suspension.<front/rear>.hardpoints.*`, `static_camber_deg`, `static_toe_deg`. 로커가 있으면 `rocker_pivot`, `rocker_axis`, `spring_rocker`, `spring_chassis` |
| 스프링·댐퍼 | `spring_rate_n_per_mm`, `motion_ratio`(로커 없을 때), `damper_n_s_per_m`(범프), `damper_rebound_n_s_per_m`, `damper_knee_mps`, `damper_high_speed_ratio` |
| 스톱·안티롤바 | `bump_travel_m`, `bump_stop_n_per_mm`, `droop_travel_m`, `anti_roll_bar_nm_per_deg` |
| 캠버 추력 | `tire.simple.camber_stiffness` (단순 모델), `.tir`는 파일의 캠버 계수(PKY6, PDY3, QDZ8 …) |

- **K&C**: 바퀴 이동 ±(범프·처짐의 1.5배)와 랙 이동 범위를 미리 풀어 표로 씁니다. 조향은 랙 이동이고, 핸들각 ↔ 랙의 비는 중앙에서 `steering.ratio`가 되도록 정합니다. 그래서 좌우 바퀴 각(애커만)과 범프 스티어가 하드포인트에서 나옵니다. L3에서는 `steering.ackermann`, `mechanical_trail_m`, `suspension.roll_center_*`가 쓰이지 않습니다.
- **정지 상태**: 시작할 때 정적 평형(타이어 처짐, 스프링 예압)에서 출발합니다. 경사·뱅크 위에서는 차체가 노면을 따라 기울어진 상태로 시작합니다.
- **4포스트 리그**: `rig.<fl/fr/rl/rr>.z`와 `.vz`(노면 변위와 속도)를 Python에서 써 넣으면 바퀴 밑 노면이 그만큼 움직입니다 (주파수 응답, 승차감 시험).

---

## 플러그인

팀 코드를 시뮬레이터 블록으로 넣습니다 ([설계 문서 §11](design.md#11-플러그인--사용자-코드-통합), SDK는 [sdk/README.md](../sdk/README.md)). 연결은 프로젝트의 `plugins/components.yaml`에 적습니다 (`project.yaml`의 `components:`가 이 파일을 가리킴).

```yaml
tv_native:
  enabled: true
  type: native                # native (.dll/.so) | fmu | python | builtin
  path: tv_cpp                # 확장자 없이 쓰면 .dll/.so를 찾음
  after: vcu                  # 이 블록 다음에 실행: driver | sensors | vcu
  # slot: vcu                 # 또는 이 블록을 대신함: vcu | driver
  period: 0.01
  params: {gain: 40}          # native: JSON으로, fmu: 같은 이름의 파라미터에, python: init(params)에
  ports:
    in:
      vx: vcu.vx_est                       # 신호 이름
      yaw_rate: {signal: sens.imu.gz, unit: deg/s}   # 플러그인이 단위를 말하지 않을 때 지정
      mode: {value: 1}                     # 고정값
    out:
      tq_fl: pt.motor.fl.torque_cmd        # 기존 신호를 덮어씀 (VCU 출력 보정 등)
      shift: tv.shift                      # 없는 신호는 새로 만듦
```

- 포트 이름이 버스 신호 이름과 같으면 연결하지 않아도 됩니다. 연결되지 않은 입력, 없는 포트 이름, 단위 종류가 다른 연결(`m/s`를 `rad/s`에)은 시작할 때 오류로 알려 줍니다.
- 플러그인 파일은 프로젝트 폴더 → `plugins/` → 환경 변수 `VSIM_PLUGIN_PATH`의 폴더(Windows는 `;`로 구분) → vsim 빌드의 `build/plugins` 순서로 찾습니다.
- 예제 프로젝트에 네 항목이 꺼진 채로 있습니다. 켜서 달려 보기:

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/skidpad_auto.yaml --set components.tv_native.enabled=true
# 같은 로직의 FMU, Python: components.tv_fmu.enabled=true, components.tv_py.enabled=true
# VCU를 C로 쓴 것으로 바꾸기: components.simple_vcu.enabled=true
```

### 종류별

| 종류 | 만드는 법 | 비고 |
|---|---|---|
| `native` | C: `vsim_plugin.h`의 함수 표 하나 ([simple_vcu.c](../sdk/examples/simple_vcu_c/simple_vcu.c)). C++: `vsim_plugin.hpp`의 클래스 + `VSIM_PLUGIN(...)` ([tv_cpp.cpp](../sdk/examples/tv_cpp/tv_cpp.cpp)) | 가장 빠름 (예제 1 µs/호출). 64비트로 빌드 |
| `fmu` | Simulink(`vsim_export`의 FMU 경로), OpenModelica·Dymola·AMESim 등의 FMI 2.0 Co-Simulation 내보내기 | `.fmu`를 그대로 두면 처음 실행할 때 임시 폴더에 풀어 씀. Model Exchange·FMI 3.0은 아직 없음. 변수 이름의 `.`·`[ ]`는 포트 이름에서 `_` |
| `python` | `vsim.plugin.Block`을 상속한 클래스 ([tv_py.py](../templates/projects/example_fs_ev/plugins/tv_py.py)), `module: 파일:클래스` | 수백 Hz까지 권장 (예제 7.5 µs/호출). vsim-core는 Python이 설치되어 있을 때만(빌드한 버전과 같은 Python DLL) |
| `builtin` | `slot: vcu`나 `slot: driver`에 기본 블록을 명시할 때 | |

- 실행 순서: 드라이버 → (after: driver) → 센서 → (after: sensors) → VCU → (after: vcu) → 랩 타이머. 같은 위치의 블록은 `order:` 값, 그다음 이름 순.
- 출력은 다음 호출까지 유지됩니다(ZOH). FMU는 입력을 넣고 한 주기 진행한 뒤 출력을 읽습니다(네이티브 블록과 같은 타이밍).

### Simulink 모델 → 플러그인 (`vsim_export`)

MATLAB에서 `sdk/simulink` 폴더를 경로에 추가하고:

```matlab
addpath('C:/vsim/sdk/simulink')
% 포트를 신호에 자동 연결하려면 신호 목록을 먼저 (한 번):
%   vsim-core --scenario <시나리오> --list-signals D:/projects/my_car_2026/vsim_signals.json
vsim_export('vcu_2026.slx', 'Project', 'D:/projects/my_car_2026', 'Name', 'vcu', 'Slot', 'vcu')
vsim_export('tv_2026.slx', 'Project', 'D:/projects/my_car_2026', 'After', 'vcu')
```

- 모델 조건: 고정 스텝 솔버, 샘플 시간 하나(= 플러그인 주기), 최상위 Inport/Outport가 스칼라(double, single, int8~32, uint8~32, boolean). 포트의 Unit을 정해 두면 vsim이 단위를 변환합니다.
- 경로 자동 선택: Embedded Coder가 있으면 ERT C 코드 + vsim 래퍼를 `plugins/<이름>/`에 만들고 CMake로 빌드합니다(CMake와 Visual Studio 필요). 없으면 FMU로 내보냅니다. `'Route', 'fmu'`로 고를 수 있습니다.
- 포트 연결: 포트 이름이 신호 이름에서 `.`을 `_`로 바꾼 것(`drv_cmd_throttle`)이면 자동, 아니면 비슷한 신호 후보를 주석으로 적고 비워 둡니다. `vsim_add_port(모델, 'drv.cmd.throttle', 'in', vsim_signals(프로젝트))`로 포트를 만들면 이름·단위가 맞게 생깁니다.
- `plugins/components.yaml`에 항목을 쓰고(같은 이름이면 교체) 모델 파일 경로·버전·SHA-256·내보낸 시각을 `source:`에 남깁니다.
- 이 개발 환경에는 MATLAB이 없어 `vsim_export.m`은 실행해 보지 못했습니다. 만드는 C 래퍼는 같은 틀(`vsim_ert_wrapper.c.in`)을 Embedded Coder 출력 모양의 모델에 붙여서 CI에서 빌드·실행 시험합니다. 처음 쓸 때 문제가 있으면 알려 주세요.

### 플러그인 페이지

`vsim-core`를 실시간으로 띄운 상태에서 `ui\plugins\index.html`을 엽니다.

- 왼쪽: 실행 중인 플러그인(파일, SHA-256, 포트 연결, 호출당 평균·최대 실행 시간과 주기 대비 막대)
- 오른쪽: `components.yaml` 편집. 파일을 끌어다 놓으면 `plugins/`에 저장하고 항목을 만듭니다. "포트 불러오기"로 포트 목록을 받고 신호를 고릅니다(목록에서 검색). **저장하고 다시 불러오기**를 누르면 시뮬레이터가 새 설정으로 다시 시작하고, 불러오지 못하면 이전 파일로 되돌리고 이유를 보여 줍니다.
- 편집 탭으로 저장하면 YAML 주석은 사라집니다. 주석을 지키려면 YAML 탭에서 고칩니다.

---

## 가상 CAN

예제 프로젝트에는 `can/fs_ev.dbc`(메시지 10개)와 `can/network.yaml`이 있고, 기본으로 꺼져 있습니다. 켜면 토크 명령과 피드백이 실제 차처럼 버스를 거칩니다.

```powershell
# 1 Mbit/s 버스, 프레임 시간·중재·노드 클록까지 (ui\can\index.html로 보기)
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_auto.yaml --set can.enabled=true --set can.fidelity=timing
# 버스가 붐빌 때 (PH-08): 우선순위가 높은 0x050 프레임 6개를 1 ms마다 더 보냄 → 부하 84 %, 토크 명령 지연 증가
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/acceleration.yaml --mode fast --log-dir runs `
  --set can.enabled=true --set can.fidelity=timing --set can.load.0.id=0x050 --set can.load.0.count=6 --set can.load.0.period_ms=1
```

| `can/network.yaml` | 내용 |
|---|---|
| `enabled` | 켜기 |
| `fidelity` | `ideal` \| `timing` \| `faults` (없으면 `realism.comms`: ideal → ideal, real → faults) |
| `buses` | `{name, bitrate, data_bitrate, dbc: [can/....dbc]}` (CAN FD는 DBC의 `VFrameFormat`이나 8바이트 초과) |
| `nodes` | DBC 노드의 `role`(controller: VCU처럼 시뮬레이터의 제어기 값을 보냄 / device: 인버터·BMS·센서), `clock_ppm`, `jitter_us`, `offset_ms`(송신 위상), `task_ms`(수신 태스크 주기: 놓침·중복 집계), `bus_off_recovery_ms` |
| `map` | `메시지: {신호: 시뮬레이터 신호}`. 보내는 노드는 그 신호 값을 담아 보내고, 받는 쪽에서는 그 신호가 받은 값으로 바뀝니다(단위 자동 변환: rpm ↔ rad/s, kW ↔ W, % ↔ - …). 단위를 바꾸려면 `{signal, unit}` |
| `timeouts` | `메시지: 초` 또는 `{after_s, then: hold \| zero}` (기본 주기의 3배, hold). 예제의 토크 명령은 50 ms 뒤 0 (명령이 끊긴 인버터) |
| `load` | 추가 트래픽 `[{bus, id, count, period_ms, length}]` (데이터로거, 게이트웨이 등) |
| `faults` | `[{type: drop \| delay \| error_rate \| bus_off \| skip, message, node, bus, probability, value_ms, jitter_ms, at_s, until_s}]` (`fidelity: faults`일 때) |
| `log` | `asc` \| `candump` \| `both` \| `none` |

- DBC: 표준·확장 ID, CAN FD, Intel·Motorola, 부호, float(`SIG_VALTYPE_`), 멀티플렉스(돌아가며 송신), `GenMsgCycleTime`, 주석. 주기가 없는 메시지(이벤트)는 아직 보내지 않습니다.
- 받는 쪽: device가 보낸 메시지는 VCU·플러그인이 실행되기 전에, controller가 보낸 메시지는 플랜트가 입력을 읽기 전에 반영됩니다. 받은 값은 다음 프레임까지 유지되고, 시뮬레이터 신호 자체(로그)는 보내는 쪽 값으로 남습니다. 받은 값은 `can.<메시지>.<신호>`로 따로 기록됩니다.
- 시간 단위: 버스는 연속 시간으로 계산하고, 받는 쪽은 1 ms 물리 스텝마다 그때까지 도착한 프레임을 봅니다.

| 신호 | 내용 |
|---|---|
| `can.<버스>.load` | 최근 100 ms 버스 부하 (0~1) |
| `can.<메시지>.<신호>` | 받은 값 (DBC 단위) |
| `can.<메시지>.latency` | 마지막 프레임의 송신 요청 → 도착 [s] |
| `can.<메시지>.age`, `.timeout`, `.lost` | 마지막 도착 후 시간, 타임아웃 중이면 1, 잃은 프레임 수 |
| `can.<메시지>.missed`, `.doubled` | 받는 controller의 `task_ms` 태스크가 새 프레임을 못 찾은 / 둘 이상 찾은 횟수 |
| `can.<노드>.tec`, `.bus_off` | 송신 에러 카운터, bus-off 중이면 1 |

`summary.json`의 `can`에 버스 부하 평균·최대, 메시지별 보냄·받음·잃음·지연(최소·평균·최대)·타임아웃이 남고, `python -m vsim.phenomena`가 CAN 타임아웃·bus-off·과부하를 이벤트로 보여 줍니다.

### CAN 네트워크 편집기

`vsim-core`를 실시간으로 띄우고 `ui\can\index.html`을 엽니다.

- 버스(비트레이트, DBC 선택, 예상 부하: 비트 스터핑 없음~최악, 실행 중 실제 부하), 노드(역할·클록·지터·태스크), 추가 트래픽과 고장
- DBC 탭: 메시지(ID, 이름, 길이, 주기, 송신 노드, 타임아웃과 그때 동작, 지연 통계)와 선택한 메시지의 신호(시작 비트, 길이, 바이트 순서, 부호, 배율, 오프셋, 범위, 단위, 받는 노드, **시뮬레이터 신호 연결**). 연결 칸은 단위가 맞으면 초록, 안 맞거나 없는 신호면 빨강
- "빈 연결 자동 제안": 메시지·신호 이름과 시뮬레이터 신호 이름이 겹치는 단어와 단위로 후보를 채움
- DBC 올리기(프로젝트 `can/`에 저장), 새 DBC, 내보내기(.dbc 다운로드). 저장하면 시뮬레이터가 다시 시작하고, 불러오지 못하면 이전 파일로 되돌립니다. 편집기로 저장한 `network.yaml`에는 주석이 남지 않습니다

## 전기 네트워크

예제 프로젝트의 `electrical/netlist.yaml`은 FSAE EV 구조(140S 축전지, AIR 두 개, 프리차지 600 Ω × DC 링크 600 µF, 인버터 두 개(앞·뒤), DC-DC, 4S LiFePO4 LV 배터리, 셧다운 회로 직렬 스위치 7개, VCU·BMS·IMU·인버터 제어보드, 펌프·팬, APPS 배선)이고 기본으로 꺼져 있습니다.

```powershell
# 프리차지된 상태로 출발 (ui\elec\index.html로 회로와 실시간 값 보기)
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_auto.yaml --set electrical.enabled=true
# 모두 꺼진 상태에서: 셧다운 회로 → 프리차지(약 1.1초) → AIR+ → 구동 (PH-16)
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/acceleration.yaml --mode fast --log-dir runs `
  --set electrical.enabled=true --set electrical.initial=off
# 고장: APPS2 접지 핀 부식 (PH-10) → APPS 불일치로 토크 차단
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/acceleration.yaml --mode fast --log-dir runs `
  --set electrical.enabled=true --set electrical.faults.0.type=contact_resistance --set electrical.faults.0.target=conn.X6.2 `
  --set electrical.faults.0.r_ohm=1500 --set electrical.faults.0.at_s=1
```

| `electrical/netlist.yaml` | 내용 |
|---|---|
| `enabled` | 켜기 |
| `level` | `E1` 저항만(커패시터 없음, 정상상태) \| `E2` 과도(커패시터·인덕터·DC-DC 응답). 없으면 E2, 시나리오가 `realism.electrical: ideal`이면 꺼짐(E0) |
| `initial` | `active`: 프리차지가 끝나고 릴레이·ECU가 전원대로 있는 상태에서 시작 \| `off`: 커패시터가 비고 모두 꺼진 상태에서 시작 |
| `dt_s` | 해석 간격 (기본 물리 스텝 1 ms, `time.dt`를 나누어떨어지게) |
| `components` | 부품 목록 `{id, type, 단자: 망 이름, 값...}`. 망 이름은 자유(GND·0은 접지). HV처럼 차체와 절연된 망은 10 MΩ으로 차체에 묶입니다 |
| `faults` | `[{type, target, at_s, until_s, r_ohm, to, intermittent: {rate_hz, duration_s}}]` |
| `harness.contact_r_ohm` | 커넥터 핀 하나의 접촉 저항 (기본 2 mΩ) |
| `layout` | 회로 편집기의 배치 (해석에는 안 씀) |

| 부품 `type` | 단자 | 주요 값 |
|---|---|---|
| `battery_model` | pos, neg | 차량 셀 모델(`vehicle.battery`)의 개방전압 + 팩 저항. 하나만. 있으면 모든 모터에 `inverter`가 있어야 함 |
| `inverter` | pos, neg | `motors: [fl, fr]`, `v_min`(이 아래면 꺼짐), `v_on`, `v_max`, `logic`(제어보드 전원 ECU), `enable`(조건, 예: 프리차지 완료) |
| `contactor`, `relay` | a, b, coil_a, coil_b | `coil_r_ohm`, `pull_in_v`, `drop_out_v`, `close_ms`, `open_ms`, `r_on_ohm`, `normally: open \| closed`, `rating_a` |
| `precharge` | supply, ref | `pack: [+, −]`, `link: [+, −]`, `pre_out`·`air_out: [망, 망]`(제어기가 닫는 출력), `threshold`, `mode: voltage \| time`, `time_s`, `timeout_s`, `air_relay`(규정 검사), `rule_ratio` |
| `dcdc` | in_pos, in_neg, out_pos, out_neg | `v_out`, `i_max_a`, `efficiency`, `r_out_ohm`, `response_s`, `v_in_min`, `v_in_on`, `enable` |
| `lv_battery` | pos, neg | `capacity_ah`, `r_ohm`, `soc`(시나리오 `initial.lv_soc`가 우선), `ocv_soc`·`ocv_v` 또는 `v_empty`·`v_full` |
| `ecu` | a, b | `i_a` 또는 `p_w`, `v_min`(리셋), `v_on`, `boot_s`, `can_node`(하나 또는 목록), `forces`(꺼진 동안 0이 되는 신호, `*` 가능) |
| `load` | a, b | `p_w` \| `i_a` \| `r_ohm`, `inrush: {factor, tau_s}`(모터 기동 전류), `control`(조건) |
| `fuse` | a, b | `rating_a`, `i2t_a2s`(기본 1.6 × 정격²), `r_ohm` |
| `wire` | a, b | `length_m` 또는 `route: [[x, y, z], ...]`(차량 좌표 m, `slack` 1.1), `area_mm2`, `material`, `pins: [X1.3]`, `rating_a`(기본: 단면적 표), `thermal` |
| `switch` | a, b | `closed`, `open_when`·`closed_when`(조건), `r_on_ohm` |
| `regulator` | in, out, gnd | `v_out`, `dropout_v`, `r_out_ohm`, `i_max_a`, `i_q_a` |
| `potentiometer` | a, b, wiper | `r_ohm`, `sensor`·`channel`(페달 센서의 전압 맵을 따름) 또는 `signal`·`ratio` |
| `adc` | a, ref, vref(선택) | `r_in_ohm`, `pull_down_ohm`, `pull_up: {to, r_ohm}`, `sensor`·`channel`(그 센서 채널이 이 전압을 읽음) |
| `resistor`, `capacitor`, `inductor`, `diode`, `voltage_source`, `current_source` | a, b / pos, neg | `r_ohm` / `c_f`, `esr_ohm`, `v0` / `l_h`, `r_ohm` / `v_f`, `r_on_ohm` / `v`, `r_ohm`, `signal` / `i_a` |

- **조건** (`control`, `enable`, `open_when`, `closed_when`): 신호 이름 하나(0.5 초과), 또는 `{signal, above | below, hysteresis, delay_s, latch}`, `{all: [...]}`, `{any: [...]}`. 예: BSPD `{all: [{signal: veh.brake.pressure, above: 20}, {signal: pt.power_elec, above: 5000}], delay_s: 0.5, latch: true}`, 팬 `{signal: pt.motor.rl.temp, above: 60, hysteresis: 5}`
- **고장 대상**: `open`(전선·저항·퓨즈·스위치·릴레이·부하·DC-DC·전원·커넥터 핀), `contact_resistance`(전선·저항·퓨즈·핀 `conn.X6.2`), `short`(망 이름이나 `<id>.<단자>`를 `to`(기본 GND)로, `r_ohm` 기본 1 mΩ), `stuck_closed`·`stuck_open`(릴레이·컨택터·스위치), `blow`(퓨즈)
- 시점: 해석은 드라이버 다음, 센서 앞에서 [t, t + 1 ms)를 풉니다(플랜트의 모터 전력과 제어기 명령은 1 ms 전 값). ECU 전원 상태는 VCU 다음, CAN 송신 전에 반영됩니다

| 신호 | 내용 |
|---|---|
| `elec.v.<망>` | 망 전압 (차체 기준). 망 이름의 `+`는 `_P`, `-`는 `_N` (`ACC+` → `elec.v.ACC_P`) |
| `elec.<id>.i`, `.p` | 부품 전류(a → b, 전원은 pos에서 나가는 방향), 전력 |
| `elec.<id>.closed`, `.coil_v`, `.closes`, `.chatter` | 릴레이·컨택터·스위치 접점, 코일 전압, 닫힌 횟수, 채터링(1초에 3번 이상) |
| `elec.<id>.on`, `.v`, `.resets` | ECU 동작 여부(리셋·부팅 중 0), 전원 전압, 리셋 횟수 |
| `elec.<id>.state`, `.ratio`, `.close_ratio`, `.violation` | 프리차지: 0 꺼짐·1 프리차지·2 닫는 중·3 완료·4 실패, DC 링크/팩 전압, AIR+가 붙을 때 비율, 규정 위반 |
| `elec.<id>.blown`, `.heat` | 퓨즈 단선, 용단 적분 / I²t |
| `elec.<id>.drop`, `.temp`, `.overload` | 전선 전압 강하, 도체 온도, 정격 온도 초과 |
| `elec.<id>.i_out`, `.p_out`, `.limited`, `.on` | DC-DC |
| `elec.bat.current`, `elec.motor.<m>.dc_voltage`, `.enabled` | 플랜트로 가는 축전지 전류, 인버터 DC 링크 전압, 구동 가능 |
| `elec.ts_voltage`, `elec.ts_active` | 인버터 DC 링크 최고 전압, 60 V 초과(TSAL) |
| `elec.sensor.<센서>.<채널>` | 배선을 거쳐 ECU ADC에 들어온 센서 전압 |

`summary.json`의 `electrical`: 이벤트(ECU 리셋, 퓨즈, 채터링, 프리차지 위반·실패, 전선 과열, DC-DC 정지), 전선별 최대·실효 전류·허용 대비 사용률·최대 강하(V, %)·최고 온도·손실, 부하·ECU·인버터 평균·최대 전력, LV 전력 예산, DC-DC 여유, ECU 최저 전압·리셋, 릴레이 닫힘 횟수, 프리차지 시간·비율, 해석기 통계. `python -m vsim.phenomena`도 같은 이벤트를 보여 줍니다. Python에서 동작점만: `vsim.electrical_operating_point(시나리오, {"electrical.enabled": "true"})`.

### 회로 편집기

`vsim-core`를 실시간으로 띄우고 `ui\elec\index.html`을 엽니다.

- 왼쪽: 부품 추가(HV·LV·배선·수동 부품), 부품 목록. 가운데: 회로도. 부품을 끌어 옮기고, **단자를 끌어 망(점)이나 다른 단자에 놓으면 연결**, 빈 곳에 놓으면 새 망. 휠로 확대, 빈 곳을 끌어 이동. 오른쪽: 선택한 부품의 단자·값·기타 설정(조건·목록·경로는 YAML)
- 값 표시: "동작점 검사"(편집 중인 회로를 저장하지 않고 풀어서 망 전압·전류·접점 상태를 그림에 표시) 또는 "실행 중"(시뮬레이터의 실시간 값). HV(60 V 이상) 주황, LV 파랑, 센서 신호 초록
- 아래 탭: 고장 주입 표, 하네스·전선(길이·단면적·저항·허용 전류·실행 중 최대 전류·사용률·강하·온도, 경로를 준 전선의 위에서 본 배치), LV 전력 예산(설계값 합계와 실행 평균, DC-DC 여유, ECU 최저 전압·리셋), 이벤트, 망 전압
- 저장하면 시뮬레이터가 다시 시작하고, 불러오지 못하는 회로(값 오류 등)는 저장하지 않고 이유를 알립니다. 편집기로 저장한 파일에는 주석이 남지 않습니다

---

## 센서와 현상 탐지

예제 차량에는 센서 12개가 있습니다 ([vehicle.yaml](../templates/projects/example_fs_ev/vehicle/vehicle.yaml)의 `sensors:`, 항목 설명은 [vehicle_parameters.md §18.1](vehicle_parameters.md#181-vehiclesensors-쓰는-법-m6)). 기본은 `realism: ideal`이라 센서가 참값을 그대로 냅니다. 실제처럼 하려면 시나리오에서 켭니다.

```yaml
realism:
  sensors: real      # 잡음, 드리프트, 지연, 양자화, 포화, 휠속 펄스, 진동
  variation: real    # 개체 편차: 스케일·바이어스·장착 각도·톱니 간격 (시드마다 다른 개체)
```

```powershell
$env:PYTHONPATH = "python"
# 센서 고장: APPS 2번 채널이 1초에 단선 → 100 ms 뒤 토크 차단
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/acceleration.yaml --mode fast `
  --set vehicle.sensors.5.faults.0.channel=ch2 --set vehicle.sensors.5.faults.0.type=open --set vehicle.sensors.5.faults.0.at_s=1
# 무슨 일이 있었는지와, 어느 비이상성 때문인지
python -m vsim.phenomena templates/projects/example_fs_ev/scenarios/acceleration.yaml --duration 4 --isolate --signals vcu.vx_est sens.imu.ax --report phenomena.md
```

- `--isolate`(= `only`)는 모든 분류를 ideal로 한 run을 기준으로 분류 하나씩만 real로 돌리고, `--isolate without`은 하나씩만 ideal로 돌립니다. 표의 값은 기준 run과의 RMS 차이입니다.
- 진동 피크를 보려면 로그를 센서 주기로 남깁니다(`--set logging.rate_hz=500`). 기본 100 Hz 로그는 그 자체로 aliasing을 만듭니다.
- 테스트 식에서 `peak_freq(sens.imu.az, 5, 100)`, `peak_amp(...)`를 쓸 수 있습니다. 예: [apps_fault.test.yaml](../templates/projects/example_fs_ev/tests/apps_fault.test.yaml)
- 센서를 끄고 참값으로 제어하려면 `--set control.use_sensors=false`, 센서를 아예 빼려면 `--set vehicle.sensors=`
- 변환기 단계(M11): 개체마다 바이어스·스케일 온도계수(`bias_tempco` [단위/K], `scale_tempco` [1/K], 기준 `reference_temp_c` 25 °C), 양 끝과 0에서 0인 S자 비선형성(`nonlinearity`, 풀스케일 대비), 히스테리시스(`hysteresis`, 폭), IMU 축간 간섭(`accel.cross_axis`, `gyro.cross_axis`, 1 % 기본)과 자이로 가속도 민감도(`gyro.g_sensitivity_dps_per_g`, 0.02). 센서 온도는 기온, 또는 `temperature_signal`(예: `pt.motor.fl.temp`)을 `thermal_tau_s`(기본 30 s) 늦게 따라갑니다. 온도계수·비선형성·축간 간섭은 `variation: real`일 때 개체마다 뽑고, 히스테리시스는 `sensors: real`일 때 들어갑니다.

| 센서 신호 | 내용 |
|---|---|
| `sens.imu.ax/ay/az` | 장착점 비력 (정지 시 az ≈ +9.81) |
| `sens.imu.gx/gy/gz` | 각속도 (센서 축) |
| `sens.ws_<바퀴>.omega`, `.pulses` | 펄스로 잰 바퀴 속도, 누적 펄스 수 |
| `sens.apps.ch1`, `.ch2` | 가속 페달 두 채널 전압 |
| `sens.bse.pressure` | 브레이크 압력 |
| `sens.gnss.x/y/speed/heading/valid` | 안테나 위치·속도·진행 방향, 수신 여부 |
| `vcu.throttle`, `vcu.brake`, `vcu.vx_est` | VCU가 센서로 판단한 페달·브레이크·차속 |
| `vcu.apps_implausible`, `vcu.bspc` | 타당성 검사로 토크를 끊는 중이면 1 |

---

## 트랙에서 달리기

### 자동 주행으로 랩 돌기

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_auto.yaml --mode realtime
```

라이브 뷰(`ui\live\index.html`)에 트랙, 주행 라인(주황), 구간선(노랑), 출발선(흰색)이 그려지고 위쪽에 랩 정보가 나옵니다. <kbd>M</kbd>을 누르면 트랙 전체가 보입니다. `--mode realtime` 없이 실행하면 헤드리스로 75초(3랩)를 1초 남짓에 끝내고 `--log-dir`를 주면 `summary.json`에 랩 기록이 남습니다.

| 시나리오 | 내용 |
|---|---|
| `acceleration.yaml` | FS 가속 75 m (출발선 0.3 m 뒤, 풀 스로틀) |
| `skidpad_auto.yaml` | 스키드패드 자동 주행 (오른쪽 2바퀴, 왼쪽 2바퀴; 구간 2·4가 기록 랩) |
| `autocross_auto.yaml` | 예제 오토크로스 3랩 자동 주행 |
| `autocross_mpc.yaml` | 같은 트랙을 MPC 드라이버로 ([MPC 드라이버](#mpc-드라이버)) |
| `drive.yaml` | 직접 운전 (트랙 없음 — 편집기에서 트랙을 보내거나 `--set track.template=autocross`) |

직접 운전하면서 트랙을 쓰려면:

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml --set track.template=autocross
```

### 시나리오의 트랙 설정

```yaml
track:
  template: autocross            # templates/tracks/autocross.track.json
  # file: ../tracks/my_track.track.json   # 또는 파일 (시나리오 파일 기준 상대 경로)
  start_distance_m: 5            # 출발선 몇 m 뒤에서 출발할지 (기본 5)

driver:
  type: pursuit                  # 자동 주행
  # line: racing                 # 주행 라인 (기본: 트랙 파일의 첫 라인, 없으면 center)
  profile: {lateral_g: 1.3, accel_g: 0.9, brake_g: 1.2, max_kph: 100}   # 속도 프로파일 한계
  # speed_kph: 40                # 속도 프로파일 대신 일정 속도
  # lookahead: {min_m: 2.5, gain_s: 0.12, max_m: 12}
```

트랙이 있으면 차는 출발선 뒤 센터라인 위에서 트랙 방향으로 출발합니다 (`initial.x_m`/`initial.y_m`을 직접 주면 그 위치).

- **닫힌 트랙**: 출발선을 처음 지날 때 1랩이 시작되고, 다시 지날 때마다 랩이 끝납니다.
- **열린 트랙**(가속, 스키드패드 등): 출발선부터 결승선까지가 한 번의 기록입니다.
- **코스 이탈**: 차 중심이 경계를 벗어나면 그 랩은 무효이고, 베스트 랩에서 빠집니다.

### 트랙 신호

| 신호 | 내용 |
|---|---|
| `track.s`, `track.n` | 센터라인 위치와 좌우 오프셋 (왼쪽 +) |
| `track.distance` | 출발선부터 달린 거리 (출발 전에는 음수) |
| `track.lap`, `track.lap_time` | 완료한 랩 수, 현재 랩 시간 |
| `track.last_lap`, `track.best_lap` | 지난 랩, 베스트 랩 (유효 랩만; 없으면 0) |
| `track.sector`, `track.last_sector` | 현재 구간, 마지막으로 끝난 구간 시간 |
| `track.off_track`, `track.off_track_count`, `track.lap_valid` | 코스 이탈 여부, 횟수, 현재 랩 유효 여부 |
| `drv.target_speed`, `drv.line_offset` | 자동 주행의 목표 속도, 주행 라인에서 벗어난 거리 |

## MPC 드라이버

```powershell
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_mpc.yaml --mode realtime
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_mpc.yaml --mode realtime --set driver.mode=race
```

라이브 뷰에 차 앞으로 MPC가 계획한 1.5~2초 경로가 점선으로 나옵니다. 헤드리스로는 45초(2랩)를 추종 모드 약 7초, 한계 주행 모드 약 17초에 끝냅니다.

| 모드 | 하는 일 | 예제 오토크로스 (베스트 랩) |
|---|---|---|
| `track` | 주행 라인과 속도 프로파일을 따라감. 사람 특성(반응 지연, 근육 지연, 핸들 속도, 손떨림)을 거침 | 20.3 s |
| `race` | 트랙과 마찰 한계 안에서 진행 거리를 최대화 (MPCC). 라인은 참고만, 사람 특성 없음 | 19.1 s |
| (참고) `pursuit` | Pure Pursuit + 속도 프로파일 | 20.8 s |

### 어떻게 동작하나

- **예측 모델**: 센터라인 기준 Frenet 좌표의 dynamic single-track (상태 s, n, Δψ, vx, vy, r / 입력 앞바퀴 조향각, 종가속도). 걷는 속도에서는 kinematic 모델로 넘어갑니다.
- **차를 아는 드라이버**: 시작할 때 차량의 타이어 모델(`simple` 또는 `.tir`)을 축 하중과 코너링 하중 이동(롤센터·롤 강성 배분)으로 평가해서 앞뒤 축의 타이어 곡선(마찰, 하중 민감도, Magic Formula B·C)을 맞추고, 그립의 95 %를 기대합니다 (`model.grip_scale`).
- **달리면서 배우기**: 잰 횡가속도·요 가속도로 앞뒤 축의 횡력을 구하고, 그 힘에 필요한 슬립각과 실제 슬립각의 비로 축별 슬립 강성(`drv.mpc.front_gain`, `drv.mpc.rear_gain`)을 6초 평균으로 고칩니다. 애커만, 컴플라이언스·롤 스티어, 토, 캠버처럼 single-track에 없는 것을 흡수합니다 (예제 차: 앞 0.87, 뒤 1.12 → 실제 차가 모델보다 언더스티어).
- **타이어 그립 느끼기**: 타이어 온도·공기압으로 그립이 바뀌면(`veh.wheel.*.tire_grip`, 1초 필터) 축별 기대 그립과 목표 속도(√그립)를 함께 낮춥니다. 끄기 `model.feel_grip: false`.
- **최적화**: 50 ms마다 예측 구간 30단계를 이전 계획 주변에서 선형화해서 QP를 풀고(내부점법), 크게 바뀌면 다시 선형화해서 최대 3번(신뢰 영역 안의 순차 QP).
- **제약**: 트랙 경계(차 반폭 + 여유), 마찰원(팔각형 |ax| + |ay| ≤ 1.25 g, |ay| ≤ g), 앞뒤 타이어 슬립각이 최대 횡력 슬립각 이하, 조향각·조향 속도·저크. 지킬 수 없으면 벌점이 큰 여유 변수로 풉니다.
- **페달**: 계획한 가속도를 공칭 역맵(모터 토크·기어비, 브레이크 게인 + 회생)으로 페달로 바꾸고, 느끼는 가속도(PI, 생각보다 덜 서면 더 밟음)와 계획 속도(PI)로 보정합니다.
- **사람 특성(추종 모드)**: 반응 지연만큼 앞의 상태를 예측해서 계획하고, 근육 지연만큼 미리 핸들을 돌립니다.
- **실패 처리**: QP가 실패하면 이전 계획을 이어 쓰고, 3번 연속이면 Pure Pursuit로 넘어갑니다 (`drv.mpc.status` 1, 2).
- 결정론: 같은 시드면 비트 단위로 같은 결과. 솔버는 시뮬레이션 스텝 안에서 동기로 풉니다.

### 설정

```yaml
driver:
  type: mpc
  mode: track                  # track | race
  # line: racing               # 따라갈 라인 (기본: 트랙 파일의 첫 번째 라인)
  # horizon_s: 1.5             # 예측 시간 (race 2.0), steps: 30, mpc_period_s: 0.05
  # aggressiveness: 0.95       # 기대 그립 중 최대로 쓰는 비율
  # margin_m: 0.3              # 트랙 경계 여유
  # profile:                   # 추종 모드의 속도 프로파일: 기대 그립 대비 비율
  #   lateral_share: 0.8
  #   brake_share: 0.55
  #   max_kph: 110
  # speed_profile: profiles/autocross.csv   # 기록한 속도 프로파일 (s_m, v_mps; 프로젝트 기준 상대 경로)
  # human: {enabled: true, delay_s: 0.15, bandwidth_hz: 2.5, steer_rate_deg_per_s: 600, noise_deg: 0.3}
  # model: {grip_scale: 0.95, learn: true, feel_grip: true, front_gain: 1, rear_gain: 1}   # mu: 1.4 → 고정 마찰
  # weights: {lateral: 3, heading: 1, speed: 0.6, steer_rate: 20, accel_rate: 0.02, progress: 7}
  # road_wheel_rate_deg_per_s: 90, max_jerk: 60
```

- 추종 모드를 빠르게 하려면 `profile.lateral_share`·`brake_share`를 올리거나(최대 `aggressiveness`), 레이싱 모드 주행에서 뽑은 프로파일을 `speed_profile`로 줍니다.
- 사람 특성을 끄면(`human.enabled: false`) 같은 라인을 정확한 로봇처럼 따라갑니다.

### 속도 프로파일 뽑기

```powershell
# 실행 기록에서 (가장 빠른 유효 랩)
python -m vsim.profiles runs\2026-10-23_120000_autocross_mpc\log.mcap -o templates\projects\example_fs_ev\profiles\autocross.csv
# 시나리오를 돌려서 바로 (레이싱 모드 주행 → 추종 모드 프로파일)
python -m vsim.profiles templates/projects/example_fs_ev/scenarios/autocross_mpc.yaml --set driver.mode=race --duration 45 -o race.csv
```

`--lap 2`로 랩을 고를 수 있고, 결과는 센터라인 1 m 간격의 `s_m, v_mps` CSV입니다. 실차 로그는 `track.s`, `veh.chassis.speed`, `track.last_lap` 신호로 바꿔 MCAP에 넣으면 같은 방법으로 씁니다.

### MPC 신호

| 신호 | 내용 |
|---|---|
| `drv.mpc.status` | 0 새로 풂, 1 이전 계획, 2 Pure Pursuit 폴백 |
| `drv.mpc.iterations`, `drv.mpc.failures` | 마지막 풀이의 QP 반복 수(순차 QP 합), 지금까지 실패한 풀이 수 |
| `drv.mpc.s`, `drv.mpc.n`, `drv.mpc.n_ref` | 드라이버가 보는 센터라인 위치, 좌우 위치, 목표 라인 |
| `drv.mpc.v_ref` | 추종 모드의 목표 속도 (race: 준정상상태 한계 속도) |
| `drv.mpc.delta_cmd`, `drv.mpc.ax_cmd` | 손이 따라갈 앞바퀴 조향각, 계획한 종가속도 |
| `drv.mpc.front_gain`, `drv.mpc.rear_gain` | 드라이버가 배운 앞뒤 축 슬립 강성 (1 = 타이어 데이터 그대로) |
| `drv.mpc.path.<0..7>.x/y` | 계획한 경로 점 (라이브 뷰 점선) |

---

## 타이어 온도와 브레이크 열

차량 파일의 `tire.thermal`, `tire.pressure`, `brakes.thermal`이 있으면 켜집니다 (예제 차량에 있음, 지우면 온도 영향 없음).

```yaml
tire:
  thermal: {optimum_c: 75, window_c: 45, grip_loss: 0.12, surface_heat_capacity_j_per_k: 350,
            core_heat_capacity_j_per_k: 3000, surface_core_w_per_k: 12, surface_air_w_per_k: 1,
            surface_air_w_per_k_per_mps: 0.3, core_air_w_per_k: 2, core_air_w_per_k_per_mps: 0.5,
            road_contact_w_per_k: 1.5, slip_heat_share: 0.6, rim_brake_w_per_k: 0.8}
  pressure: {cold_kpa: 62, cold_c: 20, optimum_kpa: 90, grip_loss: 0.5}
brakes:
  thermal: {front_heat_capacity_j_per_k: 350, rear_heat_capacity_j_per_k: 280, cooling_w_per_k: 2.0,
            cooling_w_per_k_per_mps: 0.4, emissivity_area_m2: 0.02,
            pad_mu_temp_c: [20, 120, 400, 600], pad_mu_factor: [0.85, 1.0, 1.0, 0.65]}
```

- **그립** = 노면 μ × 온도 그립 `max(min_grip, 1 − grip_loss·((T표면 − 최적)/window)²)` × 공기압 그립 `1 − grip_loss·((p − p최적)/p최적)²`. 타이어 모델의 정점 힘을 낮추고 강성은 그대로 둡니다.
- **열 흐름**: 표면 ← 미끄럼 일률 × `slip_heat_share`, 표면 → 공기(속도에 비례해 늘어남)·노면(`environment.road_temp_c`, 기본 기온)·카커스; 카커스 ← 구름 저항 일률 + 브레이크 디스크(림) − 공기.
- **공기압**: 게이지압, 절대압이 카커스 절대온도에 비례 (`cold_kpa` @ `cold_c`).
- **브레이크**: 바퀴마다 디스크 하나, 브레이크 일률(구동계가 계산한 바퀴별 제동 손실)로 가열, `(cooling + cooling_per_mps × 속도)·ΔT + 방사`로 냉각. 패드 마찰 배율은 표에서 보간해서 바퀴별 제동 토크 용량에 곱합니다.
- **시작 온도**: 타이어는 최적 온도(데워진 상태), 브레이크는 기온. 차가운 타이어로 시작하려면 `--set initial.tire_temp_c=20`, 디스크는 `initial.brake_temp_c`.

| 신호 | 내용 |
|---|---|
| `veh.wheel.<fl/fr/rl/rr>.tire_temp`, `tire_core_temp` | 트레드 표면, 카커스·내부 공기 온도 [°C] |
| `veh.wheel.<..>.tire_pressure`, `tire_grip` | 공기압 [kPa 게이지], 온도·공기압 그립 배율 |
| `veh.wheel.<..>.brake_temp`, `brake_mu` | 디스크 온도 [°C], 패드 마찰 배율 |
| `veh.brake.balance` | 앞 제동 토크 비율 (패드 온도 반영) |

예제 시험: `tire_warmup.test.yaml`(차가운 타이어 레이싱, 첫 랩 −1.4 s), `brake_fade.test.yaml`(107 → 30 km/h 15회 반복 제동, `scenarios/brake_fade.yaml`).

## 셀 편차

```yaml
battery:
  spread:
    capacity_pct: 1.5        # 직렬 그룹 용량 편차 (1σ, %)
    resistance_pct: 6        # 내부저항 편차
    soc_pct: 0.5             # 조립 시 SoC 편차
    weak_group: {capacity_pct: -8, resistance_pct: 25, soc_pct: -3}   # 0번 그룹에 더함 (선택)
```

- 표본은 `realism.variation: real`일 때만 (같은 시드 = 같은 차), `weak_group`은 항상 적용됩니다.
- 그룹 i의 SoC = 시작 SoC + 편차 + (평균 SoC 변화) / 용량 배율, RC 전압 = 평균 × 저항 배율, 전압 = OCV(SoC) − 저항 배율 × (RC + R0·I/병렬). 팩 전압·에너지 수지는 평균 그룹 그대로입니다.
- BMS 방전·충전 전류 한계는 그룹마다 셀 전압 한계에 닿는 전류의 최솟값, SoC 창은 최저·최고 그룹 SoC로 정합니다.
- 신호: `bat.cell_v_min`, `bat.cell_v_max`, `bat.soc_min`, `bat.soc_max`, `bat.weak_group`.
- 예제 시험 `cell_spread.test.yaml`: SoC 15 %에서 약한 그룹 하나로 방전 한계 76 → 58 kW, 75 m 3.52 → 3.65 s.

## 구동계 비틀림·백래시와 토크 리플

```yaml
driveline:
  - {motor: fl, wheel: fl, gear_ratio: 14.0, efficiency: 0.97,
     shaft: {stiffness_nm_per_rad: 6000, damping_nms_per_rad: 3, backlash_deg: 0.6}}   # 바퀴 기준
powertrain:
  motor:
    ripple:
      - {order: 30, amplitude_pct: 3, cogging_nm: 0.3}   # 기계 1회전당 30주기 (전기 6차 × 극쌍 5)
```

- `shaft`가 있는 바퀴는 로터(+감속기)가 자기 속도 좌표를 갖고, 바퀴와 스프링·감쇠로 이어집니다. 유격 안에서는 힘이 없고(감쇠도 접촉할 때만), 비틀림각이 상태로 들어갑니다. 스프링 에너지와 감쇠 손실이 에너지 수지에 들어갑니다.
- 리플은 로터 각도(상태)의 고조파: `(amplitude_pct × 토크 + cogging_nm) · sin(order·θ + phase)`. 시뮬레이션 간격이 담을 수 있는 주파수(`1/(8·dt)`, 1 ms면 125 Hz) 위에서는 줄어들어 사라집니다(앨리어싱 방지). 1 ms에서는 아주 낮은 속도(출발·크리프)에서만 효과가 있고, 더 보려면 `time.dt`를 줄입니다.
- 신호: `pt.shaft.<바퀴>.torque`(바퀴로 전하는 토크), `pt.shaft.<바퀴>.twist`.
- 예제: `scenarios/driveline_shuffle.yaml`(출발 → 토크 끊기 → 다시 밟기)과 `driveline_shuffle.test.yaml`: 출발 때 축 토크 −90 ~ +190 N·m(정상 82), 끊은 뒤 유격 안 덜컹거림, 슬립 제어가 진동을 슬립(1.2)으로 오인해 토크 차단, 다시 밟을 때 40 % 오버슈트.

## 바람

```yaml
environment:
  wind:
    speed_mps: 5                 # 정상풍
    from_deg: 300                # 불어오는 방향 (나침반: 0 북, 90 동)
    gusts:                       # 1-cos 돌풍: duration_s 동안 0 → speed_mps → 0
      - {at_s: 5, duration_s: 2.0, speed_mps: 12, from_deg: 0}
    turbulence: {sigma_mps: 1.5, length_m: 20}   # 성분별 표준편차, 소용돌이 길이
```

- 바람은 `env.wind.x`, `env.wind.y`(동·북, 땅에 대한 공기 속도) 신호입니다. 차는 공기에 대한 속도로 항력(흐름 방향)·다운포스·측력 `−q·side_force_area·sin(β)`을 받고, 측력은 `side_force_x_m`에 작용해 yaw 모멘트가 됩니다 (`veh.aero.airspeed`, `yaw_angle`, `side_force`, `yaw_moment`).
- 난류는 공기에 얼어붙은 소용돌이를 차가 지나가는 것으로 보고 시간 상수를 `length_m / 대기 속도`로 둔 1차 Gauss-Markov입니다(Dryden 스펙트럼의 저차 근사). 시드를 따릅니다.
- 시나리오에서 `realism.environment: ideal`을 명시하면 바람이 멈춥니다.
- 예제: `scenarios/crosswind.yaml`(100 km/h 직진 중 옆 돌풍 → 코가 바람 쪽으로 돌아 2.5 m 벗어남), MPC 외란 시험에 5 m/s 바람 + 난류.

## 차고 공력과 유효 구름 반경

```yaml
aero:
  ride_height_map:            # L3 (vehicle.model: full)
    front_mm: [-30, -20, -10, 0, 10, 20]   # 정적 차고 대비 앞축 중심선 차고 변화
    rear_mm: [-30, -20, -10, 0, 10, 20]
    cla_factor: [[...], ...]               # cla_m2에 곱함, 행 = front_mm, 열 = rear_mm
    front_ratio_shift: [[...], ...]        # front_ratio에 더함 (선택, 기본 0)
    cda_factor: [[...], ...]               # cda_m2에 곱함 (선택, 기본 1)
tire:
  rolling_radius: {pressure_mm_per_kpa: 0.02, deflection_share: 0.3}
```

- 차고는 축마다 두 바퀴의 (서스펜션 이동 + 타이어 처짐 변화)로 구한 차체-노면 높이 변화입니다. 피치는 앞뒤 차고 차이로 들어갑니다. 표 사이는 쌍선형 보간, 바깥은 가장자리 값. L2(two_track)는 (0, 0) 값을 씁니다.
- 예제 차량의 맵: 낮아질수록 다운포스가 늘다가(앞 −20 mm에서 +10 %) 그 아래에서 언더트레이가 실속(−16 %), 코가 내려가면 압력중심이 앞으로 갔다가 실속하면 뒤로 갑니다. 29 m/s 순항에서 앞뒤 약 −17 mm(다운포스 +6 %), 고속 제동 시작 때 앞이 −33 mm까지 내려가 실속합니다.
- 유효 구름 반경 = `wheel.radius_m` + `pressure_mm_per_kpa`·(공기압 − 최적 공기압) − `deflection_share`·(타이어 처짐 − 정적 처짐). 슬립과 바퀴 토크가 같은 반경을 써서 에너지 수지는 그대로이고, 휠속 센서로 차속을 추정하는 VCU는 이 차이만큼 틀립니다(차가운 타이어 −0.6 mm면 약 +0.3 %).
- 맵을 끄려면 키를 지우거나 `--set vehicle.aero.ride_height_map=off`.
- 신호: `veh.aero.ride_height_front`, `veh.aero.ride_height_rear` [m], `veh.aero.front_ratio`, `veh.wheel.<..>.rolling_radius` [m].

## 사운드

브라우저 3D 화면과 Unreal 뷰어에서 <kbd>N</kbd>으로 켜고 끕니다 (처음엔 꺼짐, 브라우저는 `?volume=0.6`으로 음량).

| 소리 | 무엇을 따르나 |
|---|---|
| 모터 윙윙 (10차, 2차 고조파) | 로터 회전수 = 바퀴 속도 × 감속비 (`VehicleInfo.gear_ratio`), 크기는 출력 |
| 기어 맞물림 (23차), 인버터 스위칭 (8 kHz) | 로터 회전수, 출력 |
| 타이어 스퀼 | 그립 정점을 넘은 슬립(슬립각 0.05 rad, 슬립률 0.06 이상), 하중이 크면 높은 음 |
| 노면 소음, 바람 | 속도 |
| 연석 진동 | 바퀴가 연석 위(`wheel_surface`) |

- Unreal: `UVsimCarSound`(USynthComponent)가 오디오 스레드에서 `vsim_car_sound.h`를 돌립니다(엔진 없이 테스트됨). 상태는 `SynthCommand`로 넘기고 40 ms에 걸쳐 바뀌어 딸깍거리지 않습니다. 시뮬레이터가 끊기면 조용해집니다.

---

## 트랙 편집기

`ui\editor\index.html`을 Chrome이나 Edge로 엽니다. 설치나 서버 없이 바로 열립니다.

| 도구 | 단축키 | 동작 |
|---|---|---|
| 선택 | <kbd>V</kbd> | 점 드래그로 이동, 빈 곳 드래그로 화면 이동, <kbd>Delete</kbd>로 점 삭제 |
| 점 추가 | <kbd>A</kbd> | 빈 곳 클릭: 마지막 점 뒤에 추가 / 트랙 위 클릭: 그 사이에 삽입 |
| 출발·결승 | <kbd>S</kbd> | 점 클릭: 출발선, <kbd>Shift</kbd>+클릭: 결승선 (열린 트랙) |
| 구간 | <kbd>K</kbd> | 점 클릭: 구간 경계 추가/제거 |
| 주행 라인 | <kbd>L</kbd> | 핸들을 옆으로 끌기, 트랙 클릭으로 핸들 추가, <kbd>Delete</kbd>로 삭제 |
| | <kbd>F</kbd>, <kbd>Ctrl+Z</kbd>/<kbd>Ctrl+Y</kbd>, <kbd>Ctrl+S</kbd> | 전체 보기, 되돌리기/다시, 저장 |

- 오른쪽 패널에서 점별 좌표·폭(좌/우)·고도·뱅크, 트랙 이름, 닫힌/열린 트랙을 바꿉니다. 뱅크는 왼쪽 가장자리가 높으면 +입니다. **고도 단면** 그래프로 오르내림을 확인합니다.
- **노면**: 노면 마찰(타이어 데이터 대비 배율, 젖은 노면 0.6~0.7), 거칠기(ISO 8608 등급, A가 좋은 포장), 트랙 밖 잔디의 마찰과 거칠기.
- **연석**: 점을 고르고 **선택한 점에 연석 추가** → 시작·끝 점, 왼쪽/오른쪽, 폭, 높이를 고칩니다. 경계 바깥에 붙고 0.5 m 간격 럼블이 있습니다. 콘은 연석 바깥에 놓입니다.
- **마찰 구간**: 시작·끝 점과 좌우 범위 n(왼쪽 +)으로 젖은 노면·오일 같은 구간을 만듭니다. 지도에 파랗게 표시됩니다.
- **최소 곡률 라인 계산**: 경계에서 "경계 여유"만큼 안쪽으로 곡률이 가장 작은 라인을 계산합니다(TORCS K1999 방식). 그다음 핸들로 다듬습니다.
- **검사**: 최소 반경 4.5 m 미만, 안쪽 경계가 접히는 곳, 폭 3 m 미만, 센터라인 교차, 15 % 넘는 경사를 알려 줍니다.
- **곡률 색**: 센터라인을 반경으로 색칠합니다 (파랑: 직선 ~ 빨강: 7 m 이하).
- **열기·저장**: `*.track.json`을 열고, 저장하면 다운로드 폴더에 받아집니다. TUMFTM racetrack-database CSV(`x_m, y_m, w_tr_right_m, w_tr_left_m`)도 열 수 있습니다.

### 그리고 바로 달리기

1. vsim-core를 realtime으로 실행합니다 (예: `drive.yaml`로 직접 운전, 또는 `autocross_auto.yaml --mode realtime`으로 자동 주행).
2. 편집기에서 **시뮬레이터로 보내기**를 누르면 vsim-core가 그 트랙으로 처음부터 다시 시작합니다. 라이브 뷰도 새 트랙으로 바뀝니다.
3. 고치고 다시 보내기를 반복합니다. **가져오기**는 시뮬레이터가 쓰고 있는 트랙을 편집기로 불러오고, **시나리오 트랙으로 되돌리기**는 원래 트랙으로 돌아갑니다.

편집기와 시뮬레이터의 스플라인 계산은 같아서(차이 1e-14 m 수준) 편집기에서 본 모양 그대로 달립니다.

### 트랙 파일 (`*.track.json`)

```json
{
 "schema_version": 1, "name": "my track", "closed": true,
 "points": [{"x": 0, "y": 0, "z": 0, "w_left": 1.75, "w_right": 1.75, "bank_deg": 0}, ...],
 "start_point": 0, "finish_point": -1, "sector_points": [12, 30],
 "lines": [{"name": "racing", "offsets": [[0.0, 0.5], [0.05, -0.8], ...]}],
 "surface": {"mu": 1.0, "roughness": "A", "grass_mu": 0.6, "grass_roughness": "C"},
 "kerbs": [{"from_point": 4, "to_point": 9, "side": "left", "width": 1.0, "height": 0.025}],
 "patches": [{"from_point": 39, "to_point": 46, "n_min": -4, "n_max": 1, "mu": 0.6}]
}
```

- 센터라인은 점들을 지나는 3차 스플라인입니다 (닫힌 트랙은 주기 스플라인).
- 출발·결승·구간은 점 번호로 지정해서, 점을 옮겨도 함께 움직입니다.
- 주행 라인은 센터라인 기준 좌우 오프셋 n [m]을 트랙 길이 비율 u ∈ [0, 1]마다 적은 것입니다. 그래서 센터라인을 고쳐도 라인이 따라갑니다.
- 고도는 점들을 지나는 스플라인, 뱅크는 점 사이 선형입니다. `surface`, `kerbs`, `patches`는 생략할 수 있고(평평하고 매끈한 노면), M2 이전 파일도 그대로 읽힙니다.
- 거칠기는 트랙 이름으로 정해지는 난수라 같은 트랙은 언제나 같은 요철입니다. 등급의 PSD Gd(n) = Gd(n0)(n/n0)^-2 (n0 = 0.1 cycle/m, A = 16e-6 m³, 등급마다 4배)를 0.01~5 cycle/m에서 따릅니다.

템플릿은 `templates/tracks/`에 있고, `python -m vsim.tracks templates/tracks`로 다시 만들 수 있습니다 ([python/vsim/tracks.py](../python/vsim/tracks.py)).

---

## 시나리오 테스트

시나리오를 돌리고 판정식으로 통과/실패를 가립니다 (설계 문서 §13.2).

```powershell
$env:PYTHONPATH = "python"
python -m vsim.testing templates/projects/example_fs_ev/tests --report report.md
```

테스트 파일(`*.test.yaml`):

```yaml
test: acceleration_75m
description: FS 가속 시험
scenario: ../scenarios/acceleration.yaml   # 테스트 파일 기준 상대 경로
overrides: {vehicle.mass_kg: 300}           # 선택
duration_s: 8                               # 선택 (기본: 시나리오의 time.t_end)
seeds: [1, 2, 3]                            # 선택 (시드마다 한 번씩 실행)
checks:
  - name: 75 m를 4.0초 안에
    expr: "final(track.last_lap) < 4.0"
  - name: 출발 후 슬립률
    expr: "max(window(vcu.motor.fl.slip, 0.5, 3.0)) < 0.2"
```

| 함수 | 뜻 |
|---|---|
| `max` `min` `mean` `std` `rms` `abs` | 통계 / 절댓값 |
| `final(x)` `initial(x)` `at(x, t)` | 마지막 값, 처음 값, 시각 t의 값 |
| `window(x, t0, t1)` | t0~t1 구간만 |
| `time_to(x, v)` | x가 처음 v에 닿는 시각 (못 닿으면 inf) |
| `rise_time(x, 목표)` `overshoot(x, 목표)` `settle_time(x, 목표, 허용비)` | 응답 특성 |
| `count(조건)` `duration(조건)` `any` `all` | 조건이 참이 된 횟수, 참인 시간 |
| `changes(x)[i]` | x가 바뀐 값들 중 i번째 (예: 랩·구간 시간) |

- 신호는 이름 그대로 씁니다 (`track.best_lap`, `veh.chassis.vx`). `+ - * /`, 비교, `and`/`or`/`not`을 쓸 수 있습니다.
- 판정식은 위 함수만 해석하고 `eval`은 쓰지 않습니다.
- 실패한 판정은 비교의 양쪽 값을 보여 줍니다. `--json`으로 JSON 리포트도 씁니다.
- 하나라도 실패하면 종료 코드 1이라서 CI에서 그대로 쓸 수 있습니다 (이 저장소의 CI도 예제 테스트를 돌립니다).

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
| `veh.wheel.<fl/fr/rl/rr>.*` | 수직 하중, 타이어 힘·모멘트, 슬립률·슬립각, 바퀴 회전속도, 조향각. L3: `travel`(범프 +), `camber`, `spring_force`, `damper_force`, `deflection`(타이어 처짐), `road_z`, `mu`, `surface`(0 노면, 1 연석, 2 잔디) |
| `veh.chassis.z/roll/pitch/heave` | L3 차체: 현가 질량 무게중심 높이, 롤(왼쪽 위 +), 피치(코 숙임 +), 정적 높이 대비 상하 |
| `veh.ffb.torque_sw` | 노면이 핸들에 주는 토크 (N·m, 반시계 +) |
| `pt.motor.<id>.*` | 토크 명령·실제, 회전속도, 가용 토크, 권선 온도, 손실, 전력 |
| `bat.*` | SoC, 단자 전압·전류·전력, OCV, 온도, BMS 한계 (`bat.limit.*`) |
| `vcu.*` | 출력 한계, 회생 비율, 바퀴별 슬립과 슬립 제어 계수, 센서로 판단한 페달·차속, 타당성 검사 상태 |
| `sens.<센서>.*` | 센서 출력 (위 "센서와 현상 탐지") |
| `can.*` | 가상 CAN: 버스 부하, 받은 신호, 메시지 지연·타임아웃 (위 "가상 CAN") |
| `elec.*` | 전기 네트워크: 망 전압, 부품 전류, 접점·ECU·퓨즈·프리차지 상태 (위 "전기 네트워크") |
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
| `--input-profiles <폴더>` | 휠 프로필 폴더 지정 (기본: 소스 트리의 `config/input`). 보정 페이지도 여기에 저장 |
| `--no-link` | 공유 메모리로 내보내지 않음 (Unreal 뷰어 안 씀) |
| `--link-name <이름>` | 공유 메모리 이름 (기본 `vsim_state_v2`). 시뮬레이터 두 개를 동시에 띄울 때 |
| `--export-track-glb <파일>` | 시나리오의 트랙을 3D 모델(.glb)로 저장하고 종료 |
| `--export-car-glb <파일>` | 시나리오의 차량 3D 모델(.glb)을 저장하고 종료 |
| `--export-kc <파일.csv>` | 하드포인트에서 푼 서스펜션 K&C 곡선을 CSV로 저장하고 정적 값(킹핀각, 캐스터, 스크럽, 롤센터 등)을 출력한 뒤 종료 |
| `--list-signals <파일.json>` | 시나리오의 신호 목록(이름, 단위, 설명)을 저장하고 종료. Simulink `vsim_export`가 포트를 연결할 때 씀 |
| `--virtual-wheel` | 가상 휠 추가 (`input.virtual` RPC로 조작, UI 시험용) |

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
├─ vehicle/builder.yaml                (차량 빌더가 저장) CAD에서 계산한 값. vehicle.yaml의 같은 항목을 덮어씀
├─ vehicle/visual/car.glb              (차량 빌더가 저장) 3D 화면의 차량 모델
├─ vehicle/cad/                        (선택) 원본 CAD 사본
├─ vehicle/tires/example_fs_10in.tir   예제 Magic Formula 타이어 파일 (합성 데이터)
├─ control/vcu.yaml                    참조 VCU 설정 (토크 분배, 회생, 슬립 제어, 출력 한계)
├─ scenarios/                          drive, keyboard, sine_steer, straight_accel,
│                                       acceleration, skidpad_auto, autocross_auto
└─ tests/                              시나리오 테스트 (*.test.yaml)
templates/tracks/                      트랙 템플릿 (*.track.json)
```

자기 차량은 이 폴더를 복사해서 값을 바꾸면 됩니다. 시나리오의 `project:`는 시나리오 파일 기준 상대 경로입니다.

| 바꾸고 싶은 것 | 위치 |
|---|---|
| 차량 모델 | `vehicle.yaml`의 `model: two_track` (L2) 또는 `single_track` (L1, 빠른 검토용) |
| 구동계 토폴로지 | `vehicle.yaml`의 `driveline:` 목록 (주석에 후륜 단일 모터 + LSD 예시) |
| 타이어를 `.tir`로 | `tire: {model: mf, file: vehicle/tires/<파일>.tir}` (축별로는 `tire.front.file`) |
| 드라이버 | 시나리오의 `driver.type`: `dil`(휠/키보드), `keyboard`, `maneuver`(스크립트), `pursuit`(자동 주행), `external`(Python) |
| 트랙 | 시나리오의 `track.template` 또는 `track.file` |
| 표준 시험 입력 | `driver.steer: {type: sweep, amplitude: 20, f0_hz: 0.2, f1_hz: 3, duration_s: 20}` 또는 `{type: sine_dwell, amplitude: 60, freq_hz: 0.7, dwell_s: 0.5}` |

---

## Linux

```bash
python3 -m pip install pybind11 numpy pytest mcap pyyaml
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
| `track.template '...': templates/tracks not found` | 시나리오가 저장소 밖에 있으면 템플릿을 못 찾습니다. `track.file`로 트랙 파일 경로를 직접 지정 |
| 편집기 "연결할 수 없습니다" | vsim-core를 realtime 모드로 실행 중인지, 주소(`ws://127.0.0.1:8765`)가 맞는지 확인 |
| `ModuleNotFoundError: No module named 'yaml'` | `python -m pip install pyyaml` (테스트 러너용) |
| 3D 화면이 검은 화면 / 아무것도 안 나옴 | 인터넷 연결 확인 (three.js를 cdn.jsdelivr.net에서 받음). 브라우저 개발자 도구(F12) 콘솔의 오류를 확인 |
| 3D 화면이 끊김 | 다른 탭을 닫고, 노트북이면 고성능 GPU로 Chrome 실행. 왼쪽 위 fps 확인 |
| `shared-memory link off: another simulator ... publishes` | vsim-core가 이미 하나 실행 중. 그쪽을 끄거나 `--link-name`을 다르게 |
| 보정 페이지 "휠 입력이 꺼져 있습니다" | `driver.type: dil` 시나리오(`drive.yaml`)를 `--no-wheel` 없이 실행 |
| `import vsim` 실패 | 빌드 후 `python/vsim/` 안에 `_vsim...pyd`(Windows) 또는 `.so`(Linux)가 있는지 확인 |
| `components.<이름>.path: '...' not found` | 플러그인 파일 위치 확인. 프로젝트 `plugins/`에 두거나 `VSIM_PLUGIN_PATH`에 폴더 추가. 예제 플러그인은 vsim을 빌드해야 생김 |
| `cannot load '....dll' (error 126)` | 플러그인이 쓰는 다른 DLL(예: MSVC 런타임, 회사 라이브러리)을 못 찾음. 플러그인과 같은 폴더에 두기. 32비트로 빌드했으면 64비트로 |
| `plugin ABI version ...` | 플러그인을 이 vsim의 `sdk/c/include/vsim_plugin.h`로 다시 빌드 |
| `inputs not connected: ...` | `components.yaml`에서 그 입력 포트에 신호나 `{value: ...}`를 지정 |
| `python plugins need python3xx.dll` | vsim-core를 빌드한 버전의 Python을 설치하고 PATH에 추가 (Python 플러그인을 안 쓰면 필요 없음) |
| FMU `no binary for this platform` | FMU를 64비트 Windows(`win64`) 또는 Linux(`linux64`) 바이너리 포함으로 다시 내보내기 |
| `can.map...: cannot convert ...` | DBC 신호 단위와 시뮬레이터 신호 단위의 종류가 다름. 연결이 맞는지 확인하거나 `{signal: ..., unit: ...}`로 DBC 쪽 단위 지정 |
| CAN을 켜니 차가 안 감 | `can.<메시지>.timeout` 확인. 토크 명령 메시지가 타임아웃이면 인버터가 0으로 (버스 과부하, bus-off, 고장 설정) |
| ASC를 CANalyzer가 못 엶 | CAN FD 줄은 ASC 형식을 간단히 씁니다. candump 로그(`can.log: candump`)를 SavvyCAN으로 |
| 전기 네트워크를 켜니 차가 안 감 | `elec.PCC.state`(3이 완료), `elec.motor.<m>.enabled`, `elec.<인버터>.v` 확인. `initial: off`면 프리차지(약 1초) 동안은 구동하지 않음. 셧다운 회로 스위치(`elec.<id>.closed`)가 열렸는지 |
| `electrical: motor rr has no inverter in the netlist` | `battery_model`이 있으면 모든 구동 모터가 어떤 `inverter`의 `motors`에 있어야 합니다 |
| `electrical.components.N (...): needs ...` | 그 부품 종류에 꼭 필요한 값이 빠짐 (위 부품 표) |
| ECU가 계속 리셋됨 | `elec.<ECU>.v` 최저값과 `v_min` 비교. LV 배터리 SoC(`initial.lv_soc`), DC-DC 동작(`elec.DCDC.on`), 배선·접지 저항, 기동 전류가 큰 부하(`inrush`) 확인 |
| `electrical.initial 'False'` | YAML 1.1 파서가 `off`를 거짓으로 읽음. `"off"`처럼 따옴표 |
