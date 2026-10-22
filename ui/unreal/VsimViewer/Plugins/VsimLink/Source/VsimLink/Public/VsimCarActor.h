// The car, built from the engine's basic shapes to the simulator's vehicle
// dimensions: body with visual roll and pitch, steered and spinning wheels,
// a turning steering wheel, and the chase / cockpit / nose cameras.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VsimTypes.h"
#include "vsim_link.h"
#include "VsimCarActor.generated.h"

class UCameraComponent;
class UMaterialInterface;
class USceneComponent;
class USpringArmComponent;
class UStaticMesh;
class UStaticMeshComponent;

UCLASS()
class VSIMLINK_API AVsimCarActor : public AActor
{
	GENERATED_BODY()

public:
	AVsimCarActor();

	/** Rebuilds the parts when the vehicle dimensions change. */
	void Configure(const vsim_link::VehicleInfo& Info);
	void ApplyState(const vsim_link::VehicleState& State);
	/** Activates the camera for Chase, Cockpit or Nose (the others use a separate camera actor). */
	void UseCamera(EVsimCamera Camera);

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<USceneComponent> Root;

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
	UStaticMeshComponent* AddPart(USceneComponent* Parent, UStaticMesh* Shape, const FVector& CentreM,
		const FVector& ScaleM, const FLinearColor& Color, const FRotator& Rotation = FRotator::ZeroRotator);
	USceneComponent* AddPivot(USceneComponent* Parent, const FVector& LocationM);
	void ClearParts();

	UPROPERTY()
	TObjectPtr<UStaticMesh> Cube;

	UPROPERTY()
	TObjectPtr<UStaticMesh> Cylinder;

	UPROPERTY()
	TObjectPtr<UStaticMesh> Sphere;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> BaseMaterial;

	UPROPERTY()
	TArray<TObjectPtr<USceneComponent>> Parts;

	UPROPERTY()
	TArray<TObjectPtr<USceneComponent>> WheelSteer;

	UPROPERTY()
	TArray<TObjectPtr<USceneComponent>> WheelSpin;

	UPROPERTY()
	TObjectPtr<USceneComponent> SteeringWheel;

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> Helmet;

	float Built[7] = {};  // dimensions the parts were built for
	double CgHeight = 0.28;
};
