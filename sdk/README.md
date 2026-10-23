# vsim 플러그인 SDK

팀 코드(VCU, 토크 벡터링, BMS 로직, 드라이버 모델 등)를 시뮬레이터의 블록으로 넣는 도구입니다. 설계는 [설계 문서 §11](../docs/design.md#11-플러그인--사용자-코드-통합), 사용법은 [build.md "플러그인"](../docs/build.md#플러그인)을 보세요.

| 폴더 | 내용 |
|---|---|
| `c/include/vsim_plugin.h` | Native 플러그인 C ABI (모든 .dll/.so 플러그인의 공통 규약) |
| `cpp/include/vsim_plugin.hpp` | C++ 헬퍼 (헤더 하나): 클래스 하나와 `VSIM_PLUGIN(...)` 한 줄 |
| `fmi2/include/vsim_fmi2.h` | FMI 2.0 형식·함수 선언 (FMU를 C로 직접 만들 때) |
| `simulink/` | `vsim_export` (Simulink 모델 → 플러그인), `vsim_signals`, `vsim_add_port` |
| `examples/simple_vcu_c/` | C로 쓴 최소 VCU (`vcu` 슬롯 교체) |
| `examples/tv_cpp/` | C++ 토크 벡터링 (VCU 다음에 실행) |
| `examples/tv_fmu/` | 같은 토크 벡터링을 FMU로 (`tv_fmu.fmu`로 묶임) |
| `../templates/projects/example_fs_ev/plugins/tv_py.py` | 같은 토크 벡터링을 Python으로 |

세 토크 벡터링 예제(C++, FMU, Python)는 같은 식이라 시뮬레이션 결과가 비트 단위로 같습니다 (`core/tests/test_plugins.cpp`, `python/tests/test_plugins.py`).

## 빌드

vsim을 빌드하면 예제도 함께 빌드되어 `build/plugins/`에 들어가고, 시뮬레이터는 경로 없이 이름(`path: tv_cpp`)으로 찾습니다. 내 플러그인은 프로젝트의 `plugins/` 폴더에 두거나 `VSIM_PLUGIN_PATH`에 폴더를 추가합니다.

C/C++ 플러그인은 아무 빌드 시스템으로나 공유 라이브러리로 만들면 됩니다. CMake 예:

```cmake
add_library(my_vcu MODULE my_vcu.c)
target_include_directories(my_vcu PRIVATE <vsim>/sdk/c/include)
set_target_properties(my_vcu PROPERTIES PREFIX "")   # my_vcu.dll / my_vcu.so
```

## 규약 요약

- 포트: 이름·단위·방향. vsim이 연결된 신호 단위와 다르면 변환합니다 (`rpm` ↔ `rad/s`, `km/h` ↔ `m/s`, `%` ↔ `-`, `degC` ↔ `K` …).
- `step(t, in, out)`을 `period`마다 부릅니다. 출력은 다음 호출까지 유지됩니다 (zero-order hold).
- 결정론: `t`(시뮬레이션 시간)를 쓰고 벽시계·시드 없는 난수를 쓰지 마세요. 같은 시나리오·시드면 결과가 비트 단위로 같아야 합니다.
- 실행마다 플러그인 파일의 SHA-256, 이름, 버전, 연결이 `meta.json`에, 호출당 실행 시간이 `summary.json`에 남습니다.
