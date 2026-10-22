// Keyboard for the viewer window: driving keys go to the simulator through
// the link (the steering wheel is read by vsim-core itself), the rest
// switches cameras and sends reset / pause.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "VsimPlayerController.generated.h"

class AVsimWorld;

UCLASS()
class VSIMLINK_API AVsimPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AVsimPlayerController();

	virtual void BeginPlay() override;
	virtual void PlayerTick(float DeltaTime) override;

	AVsimWorld* GetVsimWorld();
	bool IsHudVisible() const { return bHudVisible; }
	bool IsHelpVisible() const { return bHelpVisible; }

private:
	TWeakObjectPtr<AVsimWorld> VsimWorld;
	bool bHudVisible = true;
	bool bHelpVisible = true;
};
