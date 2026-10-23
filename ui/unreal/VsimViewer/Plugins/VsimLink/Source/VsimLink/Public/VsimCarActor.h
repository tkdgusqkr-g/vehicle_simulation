// The car as published by the simulator (vsim_link.h, kMeshCar): body,
// wheels, uprights, steering wheel and suspension links moved by the
// vehicle state, plus the chase / cockpit / nose cameras.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VsimMesh.h"
#include "VsimTypes.h"
#include "vsim_link.h"
#include "VsimCarActor.generated.h"

class UCameraComponent;
class UMaterialInstanceDynamic;
class UProceduralMeshComponent;
class USceneComponent;
class USpringArmComponent;

UCLASS()
class VSIMLINK_API AVsimCarActor : public AActor
{
	GENERATED_BODY()

public:
	AVsimCarActor();

	/** Replaces the parts with the region's (a car mesh region). */
	void Build(const vsim_link::MeshRegion& Region, const FVsimMaterialSet& Materials, bool bFlipWinding);
	/** Places the cockpit and nose cameras for the vehicle's dimensions. */
	void ConfigureCameras(const vsim_link::VehicleInfo& Info);
	void ApplyState(const vsim_link::VehicleState& State);
	/** Activates the camera for Chase, Cockpit or Nose (the others use a separate camera actor). */
	void UseCamera(EVsimCamera Camera);
	bool HasModel() const { return Parts.Num() > 0; }

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<USceneComponent> Root;

	/** Pitch and roll pivot, at the CG height. */
	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<USceneComponent> Body;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<USpringArmComponent> ChaseArm;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UCameraComponent> ChaseCamera;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UCameraComponent> CockpitCamera;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UCameraComponent> NoseCamera;

private:
	struct FCorner
	{
		USceneComponent* Steer = nullptr;  // at the hub, turns with the steering
		USceneComponent* Spin = nullptr;   // under Steer, rolls
		FVector Hub = FVector::ZeroVector;  // cm, car frame
	};
	struct FLink
	{
		UProceduralMeshComponent* Mesh = nullptr;
		FVector Inboard = FVector::ZeroVector;  // cm, body frame (origin below the CG)
		FVector Outer = FVector::ZeroVector;    // cm, relative to the hub
		int32 Corner = 0;
		bool bSteers = false;
	};
	struct FSteeringWheel
	{
		USceneComponent* Pivot = nullptr;
		FVector Axis = FVector::XAxisVector;  // Unreal frame
	};

	void ClearParts();
	UProceduralMeshComponent* NewMesh(USceneComponent* Parent, const FVector& Location);
	USceneComponent* NewPivot(USceneComponent* Parent, const FVector& Location);

	UPROPERTY()
	TArray<TObjectPtr<USceneComponent>> Parts;

	UPROPERTY()
	TArray<TObjectPtr<UMaterialInstanceDynamic>> Materials;

	FCorner Corners[4];
	TArray<FLink> Links;
	TArray<FSteeringWheel> SteeringWheels;
	double CgHeight = 0.28;  // m
	float ConfiguredFor = -1.0f;
};
