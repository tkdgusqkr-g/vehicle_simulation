# vsim Unreal 뷰어 (M3)

실행 중인 `vsim-core`를 Unreal Engine 5 화면으로 보여 주는 프로젝트입니다. 시뮬레이터와는 공유 메모리
(`core/include/vsim/link/vsim_link.h`)로 연결되고, 차량 상태를 약 1 ms마다 받습니다. 트랙 3D 모델도
시뮬레이터가 보내 주므로 맵이나 에셋을 따로 만들 필요가 없습니다. 조명, 하늘, 트랙, 차량, 카메라는 모두 코드에서
생성됩니다.

> **주의**: 이 프로젝트는 Unreal Engine이 없는 환경에서 작성했습니다. 엔진 API를 흉내 낸 스텁 헤더로 문법 검사만
> 했고, 실제 엔진에서 빌드해 본 적은 없습니다. 처음 빌드할 때 오류가 나면 오류 메시지를 그대로 알려 주세요.

## 준비

1. Epic Games Launcher에서 **Unreal Engine 5.4 이상**을 설치합니다. 프로젝트 파일은 5.5로 표시되어 있습니다.
2. **Visual Studio 2022**를 설치하면서 "C++를 사용한 게임 개발" 워크로드를 선택합니다.
   - Unreal 버전에 맞는 MSVC 툴셋과 Windows SDK가 함께 설치됩니다.
3. 엔진 버전이 5.5가 아니면 `VsimViewer.uproject`를 우클릭하고 **Switch Unreal Engine version**에서 설치한 버전을
   고릅니다.

## 빌드와 실행

1. `ui/unreal/VsimViewer/VsimViewer.uproject`를 더블클릭합니다.
2. "모듈을 다시 빌드할까요?"라고 물으면 **예**를 누릅니다. 플러그인 `VsimLink`와 게임 모듈이 컴파일됩니다.
   - 실패하면 `.uproject`를 우클릭하고 **Generate Visual Studio project files**를 실행합니다.
   - 생성된 `VsimViewer.sln`을 열고 `Development Editor` / `Win64`로 빌드하면 자세한 오류가 보입니다.
3. 시뮬레이터를 실시간 모드로 실행합니다. 예:
   ```
   vsim-core --scenario templates/projects/example_fs_ev/scenarios/drive.yaml --set track.template=autocross
   ```
4. 에디터에서 **Play**를 누릅니다. 별도 창이 낫다면 **Standalone Game**을 고릅니다.
   - 시뮬레이터가 아직 없으면 화면에 "Waiting for vsim-core"가 뜨고, 시뮬레이터를 켜면 자동으로 연결됩니다.
   - 시뮬레이터를 다시 시작해도 자동으로 다시 연결됩니다.
5. 패키징할 때는 **Platforms → Windows → Package Project**를 씁니다. 엔진 기본 도형은
   `Config/DefaultGame.ini`에서 쿡 대상으로 지정되어 있습니다.

## 조작

| 키 | 동작 |
|---|---|
| ↑/W, ↓/S, ←/A, →/D | 가속, 브레이크, 조향 (`driver.type: dil` 또는 `keyboard`) |
| 스티어링 휠 | `vsim-core`가 직접 읽습니다. 뷰어 창에 포커스가 있어도 동작합니다. |
| C | 카메라: 추적 → 콕핏 → 노즈 → TV → 위 |
| R / Space | 리셋 / 일시정지 |
| H / F1 | HUD / 도움말 표시 |
| Esc | 종료 |

## 지연 (목표 < 50 ms)

HUD의 `input -> frame` 값은 두 시간의 합입니다.
- 시뮬레이터가 마지막으로 반영한 운전 입력(휠은 1 kHz로 읽음)이 게임 스레드에 도착하기까지 걸린 시간
- 한 프레임 시간: 그 프레임이 화면에 나가기까지 걸리는 시간의 근사치

모니터 스캔아웃 지연은 포함하지 않습니다. 녹색이면 50 ms 미만입니다.

- `Config/DefaultEngine.ini`에서 `r.VSync=0`, `t.MaxFPS=0`, `r.OneFrameThreadLag=0`, `r.GTSyncType=1`을 켜 두었습니다.
  모두 지연을 줄이는 설정입니다.
- 60 Hz 모니터에서 vsync를 켜면 30–50 ms가 추가됩니다. 화면 찢김이 거슬리면 G-SYNC나 FreeSync를 쓰는 편이 낫습니다.
- NVIDIA GPU라면 엔진의 **NVIDIA Reflex** 플러그인을 켜면 더 줄어듭니다.
- 시뮬레이터 쪽 몫만 따로 재려면 `vsim-link drive throttle`을 쓰세요. 이 컨테이너에서 평균 0.6 ms였습니다.

## 문제 해결

| 증상 | 확인 |
|---|---|
| 계속 "Waiting for vsim-core" | `vsim-link wait`로 시뮬레이터가 공유 메모리를 내보내는지 확인합니다. `--no-link`로 켜지 않았는지, `--link-name`이 같은지도 봅니다. |
| 차는 보이는데 도로가 안 보임 | 레벨에 배치한 `VsimWorld`의 `bFlipTrackWinding`을 켭니다. 삼각형 감김 방향이 반대인 경우입니다. |
| 차가 거울에 비친 것처럼 반대로 돎 | 좌표 변환을 확인해야 합니다(`Public/VsimTypes.h`). 증상을 알려 주세요. |
| 너무 어두움 / 밝음 | `VsimWorld`의 Sun 세기(기본 9 lux)를 바꾸거나, Post Process Volume으로 노출을 고정합니다. |

`VsimWorld`를 레벨에 직접 배치하면 `LinkName`과 `bFlipTrackWinding`을 바꿀 수 있습니다. 배치하지 않으면 게임 모드가
기본값으로 하나를 만듭니다.

## 구조

```
VsimViewer.uproject          맵 없음: /Engine/Maps/Entry + VsimGameMode (Config/DefaultEngine.ini)
Source/VsimViewer/           빈 게임 모듈
Plugins/VsimLink/            다른 Unreal 프로젝트에 복사해 쓸 수 있는 플러그인
  Public/vsim_link.h         공유 메모리 규약. core/include/vsim/link/vsim_link.h와 같아야 합니다 (테스트가 확인).
  VsimLinkReader             공유 메모리 열기, 읽기, 키와 명령 보내기
  VsimWorld                  조명과 하늘, 트랙과 차량 갱신, 카메라, 지연 측정
  VsimTrackActor             트랙 메시 (ProceduralMeshComponent, 부분별 색)
  VsimCarActor               엔진 기본 도형으로 만든 차량, 바퀴 조향과 회전, 차체 롤과 피치
  VsimPlayerController       키 입력
  VsimHUD                    속도, 페달, 랩, 에너지, 지연
```

좌표 변환: vsim(x 동, y 북, z 위, m, 오른손 좌표계)에서 Unreal(cm, 왼손 좌표계)로 갈 때
X = 100x, Y = −100y, Z = 100z이고, yaw_UE = −yaw입니다. 이 미러링 덕분에 vsim에서 반시계로 감긴 삼각형이
Unreal에서도 앞면이 됩니다.
