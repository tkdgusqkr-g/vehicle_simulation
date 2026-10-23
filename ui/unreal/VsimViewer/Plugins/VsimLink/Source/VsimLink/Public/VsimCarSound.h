// Procedural car sound (vsim_car_sound.h) as a synth component on the car:
// the game thread hands it each vehicle state, the audio thread renders.
#pragma once

#include "CoreMinimal.h"
#include "Components/SynthComponent.h"
#include "vsim_car_sound.h"
#include "vsim_link.h"
#include "VsimCarSound.generated.h"

UCLASS(ClassGroup = (vsim))
class VSIMLINK_API UVsimCarSound : public USynthComponent
{
	GENERATED_BODY()

public:
	/** Follows this state from the next audio buffer on. */
	void SetState(const vsim_link::VehicleState& State);
	/** Starts or stops the sound; returns whether it plays now. */
	bool Toggle();

	/** Output level, 0..1. */
	UPROPERTY(EditAnywhere, Category = "vsim")
	float Volume = 0.6f;

protected:
	virtual bool Init(int32& SampleRate) override;
	virtual int32 OnGenerateAudio(float* OutAudio, int32 NumSamples) override;

private:
	// Audio thread only (written through SynthCommand).
	vsim_sound::CarSynth Synth;
	vsim_sound::Input Current;
};
