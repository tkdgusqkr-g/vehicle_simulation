#include "VsimPlayerController.h"

#include "EngineUtils.h"
#include "InputCoreTypes.h"
#include "VsimWorld.h"
#include "vsim_link.h"

AVsimPlayerController::AVsimPlayerController()
{
	bAutoManageActiveCameraTarget = false;  // AVsimWorld picks the view
	bShowMouseCursor = false;
}

void AVsimPlayerController::BeginPlay()
{
	Super::BeginPlay();
	SetInputMode(FInputModeGameOnly());
}

AVsimWorld* AVsimPlayerController::GetVsimWorld()
{
	if (!VsimWorld.IsValid())
	{
		for (TActorIterator<AVsimWorld> It(GetWorld()); It; ++It)
		{
			VsimWorld = *It;
			break;
		}
	}
	return VsimWorld.Get();
}

void AVsimPlayerController::PlayerTick(float DeltaTime)
{
	Super::PlayerTick(DeltaTime);
	AVsimWorld* Vsim = GetVsimWorld();
	if (!Vsim)
	{
		return;
	}
	// Sent every frame: the simulator releases keys that stop coming.
	uint32 Keys = 0;
	if (IsInputKeyDown(EKeys::Up) || IsInputKeyDown(EKeys::W))
	{
		Keys |= vsim_link::kKeyThrottle;
	}
	if (IsInputKeyDown(EKeys::Down) || IsInputKeyDown(EKeys::S))
	{
		Keys |= vsim_link::kKeyBrake;
	}
	if (IsInputKeyDown(EKeys::Left) || IsInputKeyDown(EKeys::A))
	{
		Keys |= vsim_link::kKeyLeft;
	}
	if (IsInputKeyDown(EKeys::Right) || IsInputKeyDown(EKeys::D))
	{
		Keys |= vsim_link::kKeyRight;
	}
	Vsim->SendKeys(Keys);

	if (WasInputKeyJustPressed(EKeys::C))
	{
		Vsim->CycleCamera();
	}
	if (WasInputKeyJustPressed(EKeys::R))
	{
		Vsim->SendCommand(vsim_link::kCommandReset);
	}
	if (WasInputKeyJustPressed(EKeys::P) || WasInputKeyJustPressed(EKeys::SpaceBar))
	{
		Vsim->SendCommand(vsim_link::kCommandTogglePause);
	}
	if (WasInputKeyJustPressed(EKeys::H))
	{
		bHudVisible = !bHudVisible;
	}
	if (WasInputKeyJustPressed(EKeys::N))
	{
		Vsim->ToggleSound();
	}
	if (WasInputKeyJustPressed(EKeys::F1))
	{
		bHelpVisible = !bHelpVisible;
	}
	if (WasInputKeyJustPressed(EKeys::Escape))
	{
		ConsoleCommand(TEXT("quit"));
	}
}
