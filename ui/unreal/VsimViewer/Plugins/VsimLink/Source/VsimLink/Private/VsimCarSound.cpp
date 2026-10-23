#include "VsimCarSound.h"

bool UVsimCarSound::Init(int32& SampleRate)
{
	NumChannels = 1;
	Synth.SetSampleRate(static_cast<float>(SampleRate));
	return true;
}

int32 UVsimCarSound::OnGenerateAudio(float* OutAudio, int32 NumSamples)
{
	Synth.Render(Current, OutAudio, NumSamples);
	return NumSamples;
}

void UVsimCarSound::SetState(const vsim_link::VehicleState& State)
{
	bool bOnKerb = false;
	for (const uint8 Surface : State.wheel_surface)
	{
		bOnKerb = bOnKerb || Surface == 1;
	}
	const vsim_sound::Input Input = vsim_sound::FromState(State, bOnKerb);
	SynthCommand([this, Input]() { Current = Input; });
}

bool UVsimCarSound::Toggle()
{
	if (IsPlaying())
	{
		Stop();
		return false;
	}
	SetVolumeMultiplier(FMath::Clamp(Volume, 0.0f, 1.0f));
	Start();
	return true;
}
