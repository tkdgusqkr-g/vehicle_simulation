# vehicle_simulation

전기 포뮬러 차량 풀카 시뮬레이터. 실제와 같은 환경(노면·센서·전기·통신 외란)에서 차량을 모델링해서, 실제 차량에서 생길 현상을 미리 확인하는 것이 목표입니다.

- 설계: [docs/design.md](docs/design.md)
- 빌드와 실행: [docs/build.md](docs/build.md)
- 차량 데이터 요청 목록: [docs/vehicle_parameters.md](docs/vehicle_parameters.md)

## 현재 상태: M0 (굴러간다)

C++ 코어, single-track 차량 모델, 키보드 주행(브라우저 라이브 뷰), 헤드리스 실행, Python 바인딩, MCAP 로그, 결정론 테스트.

```powershell
python -m pip install pybind11 numpy pytest mcap
cmake -S . -B build
cmake --build build --config Release
build\core\Release\vsim-core.exe --scenario templates/projects/example_fs_ev/scenarios/keyboard.yaml
# 그다음 ui\live\index.html을 브라우저로 열고 방향키로 주행
```
