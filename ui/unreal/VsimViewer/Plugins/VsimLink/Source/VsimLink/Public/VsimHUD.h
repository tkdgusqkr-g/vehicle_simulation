// Speed, pedals, lap times, energy and latency drawn on the canvas.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "VsimHUD.generated.h"

UCLASS()
class VSIMLINK_API AVsimHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;

private:
	void Panel(float X, float Y, float W, float H);
	double SmoothedFps = 0.0;
};
