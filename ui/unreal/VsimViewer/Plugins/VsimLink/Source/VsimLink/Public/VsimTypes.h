// Frame conversion between vsim and Unreal, shared by the viewer classes.
//
// vsim: x east, y north, z up, metres, right-handed, yaw counter-clockwise.
// Unreal: X forward, Y right, Z up, centimetres, left-handed. Mirroring y
// makes the frames agree: X = 100 x, Y = -100 y, Z = 100 z, and an angle
// that is counter-clockwise in vsim is negative yaw in Unreal.
#pragma once

#include "CoreMinimal.h"

namespace VsimFrame
{
	inline FVector Position(double X, double Y, double Z) { return FVector(100.0 * X, -100.0 * Y, 100.0 * Z); }
	inline FVector Direction(double X, double Y, double Z) { return FVector(X, -Y, Z); }
	inline double Degrees(double Radians) { return FMath::RadiansToDegrees(Radians); }
	// Yaw (counter-clockwise from east) to an Unreal rotator.
	inline FRotator Heading(double Yaw) { return FRotator(0.0, -Degrees(Yaw), 0.0); }
}

enum class EVsimCamera : uint8
{
	Chase,
	Cockpit,
	Nose,
	Tv,
	Top,
	Count
};

inline const TCHAR* VsimCameraName(EVsimCamera Camera)
{
	switch (Camera)
	{
	case EVsimCamera::Chase: return TEXT("chase");
	case EVsimCamera::Cockpit: return TEXT("cockpit");
	case EVsimCamera::Nose: return TEXT("nose");
	case EVsimCamera::Tv: return TEXT("TV");
	case EVsimCamera::Top: return TEXT("top");
	default: return TEXT("?");
	}
}
