// The track scenery published by the simulator (asphalt, lines, cones,
// ground) as a procedural mesh, one section and colour per part.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "vsim_link.h"
#include "VsimTrackActor.generated.h"

class UMaterialInstanceDynamic;
class UMaterialInterface;
class UProceduralMeshComponent;

UCLASS()
class VSIMLINK_API AVsimTrackActor : public AActor
{
	GENERATED_BODY()

public:
	AVsimTrackActor();

	/** Replaces the mesh with the region's sections. Returns false if the region is inconsistent. */
	bool Build(const vsim_link::TrackRegion& Region, bool bFlipWinding);
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
	TObjectPtr<UMaterialInterface> BaseMaterial;

	UPROPERTY()
	TArray<TObjectPtr<UMaterialInstanceDynamic>> Materials;

	FString Name;
	TArray<FVector> RightEdge;  // asphalt rows, cm
	TArray<FVector> LeftEdge;
};
