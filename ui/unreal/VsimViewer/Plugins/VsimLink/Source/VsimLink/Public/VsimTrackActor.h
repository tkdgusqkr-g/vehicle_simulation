// The track and its scenery as published by the simulator (asphalt, lines,
// cones, ground, trees, grandstand, tents, gantry): one procedural mesh,
// one section and material per part.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VsimMesh.h"
#include "vsim_link.h"
#include "VsimTrackActor.generated.h"

class UMaterialInstanceDynamic;
class UProceduralMeshComponent;

UCLASS()
class VSIMLINK_API AVsimTrackActor : public AActor
{
	GENERATED_BODY()

public:
	AVsimTrackActor();

	/** Replaces the mesh with the region's sections. */
	void Build(const vsim_link::MeshRegion& Region, const FVsimMaterialSet& Materials, bool bFlipWinding);
	void Clear();

	/**
	 * Camera spots beside the track every Spacing metres, Offset metres
	 * outside the right edge at Height metres (Unreal units, cm).
	 */
	TArray<FVector> CameraSpots(double Spacing = 45.0, double Offset = 7.0, double Height = 3.5) const;

	const FString& TrackName() const { return Name; }

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UProceduralMeshComponent> Mesh;

private:
	UPROPERTY()
	TArray<TObjectPtr<UMaterialInstanceDynamic>> Materials;

	FString Name;
	TArray<FVector> RightEdge;  // asphalt rows, cm
	TArray<FVector> LeftEdge;
};
