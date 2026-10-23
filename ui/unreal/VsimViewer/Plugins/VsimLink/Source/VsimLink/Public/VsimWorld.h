// Connects to the simulator and keeps the scene in step with it: lights and
// sky, the track mesh, the car, the camera modes, latency measurement.
// The game mode spawns one; it can also be placed in a level.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VsimLinkReader.h"
#include "VsimMesh.h"
#include "VsimTypes.h"
#include "vsim_link.h"
#include "VsimWorld.generated.h"

class ACameraActor;
class APlayerController;
class AVsimCarActor;
class AVsimTrackActor;
class UDirectionalLightComponent;
class UExponentialHeightFogComponent;
class UMaterialInterface;
class UVolumetricCloudComponent;
class USkyAtmosphereComponent;
class USkyLightComponent;
class UStaticMeshComponent;

UCLASS()
class VSIMLINK_API AVsimWorld : public AActor
{
	GENERATED_BODY()

public:
	AVsimWorld();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Shared-memory name; vsim-core --link-name must match. */
	UPROPERTY(EditAnywhere, Category = "vsim")
	FString LinkName = TEXT("vsim_state_v2");

	/** Set if the road and car are invisible from outside (triangle winding the other way round). */
	UPROPERTY(EditAnywhere, Category = "vsim")
	bool bFlipWinding = false;

	// For the HUD and the player controller.
	bool IsConnected() const { return bConnected; }
	const vsim_link::VehicleState& GetState() const { return State; }
	double GetInputLatencyMs() const { return InputLatencyMs; }
	double GetFrameMs() const { return FrameMs; }
	FString GetTrackName() const;
	EVsimCamera GetCamera() const { return Camera; }
	void CycleCamera();
	void SendKeys(uint32 Keys);
	void SendCommand(vsim_link::Command Command);

private:
	void UpdateTrack();
	void UpdateCar();
	FVsimMaterialSet MaterialSet() const;
	void UpdateCamera(float DeltaSeconds);
	APlayerController* Player() const;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UDirectionalLightComponent> Sun;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<USkyLightComponent> SkyLight;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<USkyAtmosphereComponent> Atmosphere;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UExponentialHeightFogComponent> Fog;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UVolumetricCloudComponent> Clouds;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> PbrMaterial;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> PbrTwoSidedMaterial;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> PaintMaterial;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> BasicMaterial;

	UPROPERTY(VisibleAnywhere, Category = "vsim")
	TObjectPtr<UStaticMeshComponent> Ground;

	UPROPERTY()
	TObjectPtr<AVsimCarActor> Car;

	UPROPERTY()
	TObjectPtr<AVsimTrackActor> Track;

	UPROPERTY()
	TObjectPtr<ACameraActor> FreeCamera;  // TV and top views

	TUniquePtr<FVsimLinkReader> Reader;
	vsim_link::VehicleState State{};
	bool bConnected = false;
	uint64 TrackGeneration = 0;
	uint64 CarGeneration = 0;
	TArray<FVector> TvSpots;
	EVsimCamera Camera = EVsimCamera::Chase;
	double InputLatencyMs = 0.0;
	double FrameMs = 0.0;
};
