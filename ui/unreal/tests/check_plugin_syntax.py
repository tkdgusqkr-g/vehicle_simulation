"""Compiles the VsimLink plugin sources against ue_stub.h, a small stand-in
for the Unreal Engine API they use. Only syntax and types are checked (no
engine here); the point is to notice when vsim_link.h or the plugin code
breaks before someone builds it in Unreal.

usage: check_plugin_syntax.py <c++ compiler>
"""
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
PLUGIN = HERE.parent / "VsimViewer" / "Plugins" / "VsimLink" / "Source" / "VsimLink"
GAME = HERE.parent / "VsimViewer" / "Source" / "VsimViewer"

ENGINE_HEADERS = """CoreMinimal.h GameFramework/Actor.h GameFramework/PlayerController.h GameFramework/HUD.h
GameFramework/GameModeBase.h GameFramework/SpringArmComponent.h HAL/PlatformTime.h
Materials/MaterialInstanceDynamic.h Materials/MaterialInterface.h ProceduralMeshComponent.h
UObject/ConstructorHelpers.h Camera/CameraComponent.h Camera/CameraActor.h Components/SceneComponent.h
Components/StaticMeshComponent.h Components/DirectionalLightComponent.h
Components/ExponentialHeightFogComponent.h Components/SkyAtmosphereComponent.h Components/SkyLightComponent.h
Engine/StaticMesh.h Engine/World.h Engine/Canvas.h Engine/Engine.h Engine/Font.h EngineUtils.h
InputCoreTypes.h Modules/ModuleManager.h""".split()


def main() -> int:
    compiler = sys.argv[1] if len(sys.argv) > 1 else "c++"
    with tempfile.TemporaryDirectory() as tmp:
        include = pathlib.Path(tmp)
        stub = (HERE / "ue_stub.h").as_posix()
        generated = [f"{p.stem}.generated.h" for p in (PLUGIN / "Public").glob("*.h")]
        for header in ENGINE_HEADERS + generated:
            path = include / header
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f'#pragma once\n#include "{stub}"\n')
        sources = sorted((PLUGIN / "Private").glob("*.cpp")) + [GAME / "VsimViewer.cpp"]
        failed = 0
        for source in sources:
            command = [compiler, "-std=c++20", "-fsyntax-only", "-Wall", "-Wextra", "-Wshadow",
                       "-Wno-unused-parameter", "-Werror", f"-I{include}", f"-I{PLUGIN / 'Public'}", str(source)]
            result = subprocess.run(command, capture_output=True, text=True)
            status = "ok" if result.returncode == 0 else "FAILED"
            print(f"{status:6} {source.relative_to(HERE.parent)}")
            if result.returncode != 0:
                print(result.stdout + result.stderr)
                failed += 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
