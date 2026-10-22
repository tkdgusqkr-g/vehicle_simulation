# vehicle_simulation

전기 포뮬러 차량 풀카 시뮬레이터. 실제와 같은 환경(노면·센서·전기·통신 외란)에서 차량을 모델링해서, 실제 차량에서 생길 현상을 미리 확인하는 것이 목표입니다.

- 설계: [docs/design.md](docs/design.md)
- 빌드와 실행: [docs/build.md](docs/build.md)
- 차량 데이터 요청 목록: [docs/vehicle_parameters.md](docs/vehicle_parameters.md)

## 현재 상태: M1 (차처럼 구른다)

L2 차량 모델(네 바퀴 타이어·하중 이동·휠스핀), 구동계(4WD 인휠, 디퍼렌셜), 모터·배터리 모델, 참조 VCU(회생, 트랙션 컨트롤), **로지텍 G29 휠 + 포스 피드백**, 키보드 주행, `.tir` 타이어, 헤드리스 실행, Python 바인딩, MCAP 로그, 결정론·에너지 수지 테스트.

```powershell
python -m pip install pybind11 numpy pytest mcap
cmake -S . -B build
cmake --build build --config Release
build\core\Release\vsim-input.exe calibrate      # G29 축·FFB 방향 보정 (처음 한 번, 권장)
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/drive.yaml
# 그다음 ui\live\index.html을 브라우저로 열고 G29(없으면 방향키)로 주행
```
