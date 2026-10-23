# vehicle_simulation

전기 포뮬러 차량 풀카 시뮬레이터. 실제와 같은 환경(노면·센서·전기·통신 외란)에서 차량을 모델링해서, 실제 차량에서 생길 현상을 미리 확인하는 것이 목표입니다.

- 설계: [docs/design.md](docs/design.md)
- 빌드와 실행: [docs/build.md](docs/build.md)
- 차량 데이터 요청 목록: [docs/vehicle_parameters.md](docs/vehicle_parameters.md)

## 현재 상태: M12 (기판까지) — 로드맵 M0~M12 완료

- **M12**: 기판·ECU 연결. 회로도에서 내보낸 SPICE 넷리스트를 ngspice로 함께 풀기(예: APPS 입력 회로의 클램프와 필터), 전기 네트워크를 SPICE로 내보내기(`--export-spice`), 외부 프로그램을 lockstep으로 붙이는 가상 ECU 자리(`type: external`, C 헤더·Python 클라이언트, CAN 프레임까지), 실제 CAN 장치(SocketCAN, PCAN-Basic, Kvaser, UDP)로 책상 위 ECU를 가상 차에 연결(rest-bus)
- **M11**: 디테일 마감. 타이어 표면·카커스 온도와 공기압(첫 랩이 느림), 브레이크 디스크 온도와 패드 페이드, 바람·돌풍·난류, 셀 편차(약한 셀이 출력 제한), 구동계 비틀림·백래시와 토크 리플, 차고 공력 맵(지면 효과·실속), 유효 구름 반경, 센서 온도 드리프트·비선형성·축간 간섭, 모터 Level B(dq축 PMSM + FOC 16 kHz), 브라우저·Unreal 사운드, 실측 상관성 도구(`python -m vsim.correlate`: 입력 재생, 비교, 파라미터·센서 잡음 식별)
- **M10**: MPC 드라이버. 차량의 타이어 데이터로 자기 모델을 맞추고 달리면서 앞뒤 축의 반응을 배우는 예측 제어 드라이버. 추종 모드(주행 라인·속도 프로파일을 사람처럼: 반응 지연·근육 지연·손떨림)와 한계 주행 모드(MPCC, 진행 거리 최대화). 외란을 모두 켠 서킷에서 두 모드 완주, 속도 프로파일 추출(`python -m vsim.profiles`), 라이브 뷰에 계획 경로
- **M9**: 전기 네트워크. 고전압(축전지·AIR·프리차지·DC 링크·인버터), 셧다운 회로, 저전압(LV 배터리·DC-DC·ECU·펌프·팬), 센서 배선을 넷리스트로 정하고 매 스텝 회로 해석(MNA). 프리차지 순서·위반·채터링, ECU 브라운아웃 리셋과 CAN 노드 정지, 배선 고장 → APPS 차단, 전선 강하·온도·LV 전력 예산. 회로 편집기(`ui/elec/index.html`)
- **M8**: 가상 CAN. DBC로 정한 버스로 토크 명령·인버터 피드백·BMS·IMU가 오가고, 프레임 길이(비트 스터핑)·ID 중재·노드 클록 오차·지터·프레임 손실·bus-off·타임아웃이 제어에 그대로 영향을 줌. ASC·candump 버스 로그, CAN 네트워크 편집기(`ui/can/index.html`)
- **M7**: 플러그인. 팀 코드(C/C++ 공유 라이브러리, FMU, Python, Simulink → `vsim_export`)를 VCU·드라이버 자리나 VCU 다음에 넣고 포트를 신호에 연결(단위 자동 변환). 플러그인 페이지(`ui/plugins/index.html`), SDK와 예제(`sdk/`)
- **M6**: 센서 신호 체인(IMU 레버암·중력·장착 오차·진동 aliasing, 휠속 펄스, APPS 2채널, 브레이크 압력, GNSS 등)과 고장 주입. VCU가 센서 값으로 동작(FSAE APPS·BSPC 타당성 검사, 휠속 + IMU 차속 추정). 현상 탐지와 원인 분리(`python -m vsim.phenomena`)
- **M5**: L3 14자유도 차량 모델(차체 상하·롤·피치, 바퀴 상하, 스프링·댐퍼·스톱·안티롤바), 하드포인트에서 푼 K&C(`--export-kc`), 노면이 핸들로 느껴지는 FFB, 3D 노면(고도·뱅크·연석·ISO 8608 거칠기·젖은 구간·잔디)과 3D 트랙·지형, 새 템플릿 `hill_circuit`
- **M4**: 차량 빌더(`ui/builder/index.html`): STEP·glTF·STL을 올려서 질량·관성·무게중심·하드포인트·센서를 정하고 프로젝트에 저장하면 그 차로 주행. FSAE 규정 기준 예제 차량과 규정 점검(`python -m vsim.rules`), 하늘·그림자·PBR 재질·트랙 주변 환경·상세 차량 모델로 그래픽 개선
- **M3**: 3D 주행 화면. 브라우저(`ui/view3d/index.html`)와 Unreal Engine 5 뷰어(`ui/unreal`, 공유 메모리 연결, 직접 빌드 필요). 트랙 3D 모델 자동 생성(`.glb` 내보내기), 입력 보정 페이지(`ui/input/index.html`), 입력 → 화면 지연 측정
- **M2**: 트랙 템플릿과 2D 트랙 편집기(`ui/editor/index.html`, 그린 트랙을 실행 중인 시뮬레이터로 바로 전송), 랩·구간 타이밍, Pure Pursuit 자동 주행, 시나리오 테스트 러너(`python -m vsim.testing`)
- **M1**: L2 차량 모델(네 바퀴 타이어·하중 이동·휠스핀), 구동계(4WD 인휠, 디퍼렌셜), 모터·배터리 모델, 참조 VCU(회생, 트랙션 컨트롤), **로지텍 G29 휠 + 포스 피드백**, `.tir` 타이어
- **M0**: C++ 코어, 키보드 주행, 헤드리스 실행, Python 바인딩, MCAP 로그, 결정론

```powershell
python -m pip install pybind11 numpy pytest mcap pyyaml
cmake -S . -B build
cmake --build build --config Release
# 자동 주행으로 오토크로스 랩 (ui\live\index.html을 열어서 보기)
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_auto.yaml --mode realtime
# MPC 드라이버: 사람처럼 추종 / 한계 주행 (--set driver.mode=race)
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/autocross_mpc.yaml --mode realtime
# 직접 운전 (G29 또는 키보드): ui\view3d\index.html(3D) 또는 Unreal 뷰어로 보기
# 트랙 편집기(ui\editor\index.html)에서 트랙을 보내면 바로 그 트랙에서 주행, 휠 보정은 ui\input\index.html
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml --set track.template=autocross
# 내 차 올리기: ui\builder\index.html (시뮬레이터가 켜져 있으면 저장 후 바로 그 차로 재시작)
# 플러그인(팀 VCU·토크 벡터링 코드) 연결: ui\plugins\index.html, 또는 --set components.tv_native.enabled=true
# 가상 CAN: --set can.enabled=true (편집기 ui\can\index.html)
# 시나리오 테스트, 현상 탐지
$env:PYTHONPATH = "python"; python -m vsim.testing templates/projects/example_fs_ev/tests
python -m vsim.phenomena templates/projects/example_fs_ev/scenarios/acceleration.yaml --isolate
```
