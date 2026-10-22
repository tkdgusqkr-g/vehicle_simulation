// Default game mode of the viewer project (Config/DefaultEngine.ini): no
// pawn, the vsim player controller and HUD, and an AVsimWorld in the level.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "VsimGameMode.generated.h"

UCLASS()
class VSIMLINK_API AVsimGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AVsimGameMode();
	virtual void BeginPlay() override;
};
