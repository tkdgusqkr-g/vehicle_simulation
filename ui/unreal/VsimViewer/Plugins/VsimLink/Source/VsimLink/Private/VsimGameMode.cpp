#include "VsimGameMode.h"

#include "EngineUtils.h"
#include "VsimHUD.h"
#include "VsimPlayerController.h"
#include "VsimWorld.h"

AVsimGameMode::AVsimGameMode()
{
	PlayerControllerClass = AVsimPlayerController::StaticClass();
	HUDClass = AVsimHUD::StaticClass();
	DefaultPawnClass = nullptr;  // the view follows the simulated car
}

void AVsimGameMode::BeginPlay()
{
	Super::BeginPlay();
	// A level may already contain one (to set LinkName or bFlipTrackWinding).
	if (TActorIterator<AVsimWorld>(GetWorld()))
	{
		return;
	}
	GetWorld()->SpawnActor<AVsimWorld>(AVsimWorld::StaticClass(), FTransform::Identity);
}
