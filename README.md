# vehicle_simulation

전기 포뮬러 차량 풀카 시뮬레이터. 실제와 같은 환경(노면·센서·전기·통신 외란)에서 차량을 모델링해서, 실제 차량에서 생길 현상을 미리 확인하는 것이 목표입니다.

- 설계: [docs/design.md](docs/design.md)
- 빌드와 실행: [docs/build.md](docs/build.md)
- 차량 데이터 요청 목록: [docs/vehicle_parameters.md](docs/vehicle_parameters.md)

## 현재 상태: M7 (실차 코드로 달린다)

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
# 직접 운전 (G29 또는 키보드): ui\view3d\index.html(3D) 또는 Unreal 뷰어로 보기
# 트랙 편집기(ui\editor\index.html)에서 트랙을 보내면 바로 그 트랙에서 주행, 휠 보정은 ui\input\index.html
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml --set track.template=autocross
# 내 차 올리기: ui\builder\index.html (시뮬레이터가 켜져 있으면 저장 후 바로 그 차로 재시작)
# 플러그인(팀 VCU·토크 벡터링 코드) 연결: ui\plugins\index.html, 또는 --set components.tv_native.enabled=true
# 시나리오 테스트, 현상 탐지
$env:PYTHONPATH = "python"; python -m vsim.testing templates/projects/example_fs_ev/tests
python -m vsim.phenomena templates/projects/example_fs_ev/scenarios/acceleration.yaml --isolate
```
