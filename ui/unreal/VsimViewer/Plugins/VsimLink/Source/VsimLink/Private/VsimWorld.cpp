#include "VsimWorld.h"

#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"
#include "VsimCarActor.h"
#include "VsimCarSound.h"
#include "VsimTrackActor.h"

AVsimWorld::AVsimWorld()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	USceneComponent* Origin = CreateDefaultSubobject<USceneComponent>(TEXT("Origin"));
	Origin->SetMobility(EComponentMobility::Movable);
	RootComponent = Origin;

	// Daylight that needs no baking: movable sun, real-time sky capture,
	// physical atmosphere, light haze.
	Sun = CreateDefaultSubobject<UDirectionalLightComponent>(TEXT("Sun"));
	Sun->SetupAttachment(Origin);
	Sun->SetMobility(EComponentMobility::Movable);
	Sun->SetRelativeRotation(FRotator(-38.0, -35.0, 0.0));
	Sun->SetIntensity(9.0f);
	Sun->SetAtmosphereSunLight(true);
	Sun->SetCastShadows(true);

	SkyLight = CreateDefaultSubobject<USkyLightComponent>(TEXT("SkyLight"));
	SkyLight->SetupAttachment(Origin);
	SkyLight->SetMobility(EComponentMobility::Movable);
	SkyLight->bRealTimeCapture = true;
	SkyLight->SourceType = ESkyLightSourceType::SLS_CapturedScene;

	Atmosphere = CreateDefaultSubobject<USkyAtmosphereComponent>(TEXT("Atmosphere"));
	Atmosphere->SetupAttachment(Origin);

	Fog = CreateDefaultSubobject<UExponentialHeightFogComponent>(TEXT("Fog"));
	Fog->SetupAttachment(Origin);
	Fog->SetFogDensity(0.003f);

	// Volumetric clouds with the engine's default cloud material.
	Clouds = CreateDefaultSubobject<UVolumetricCloudComponent>(TEXT("Clouds"));
	Clouds->SetupAttachment(Origin);

	// Grass to the horizon under the track's own ground.
	Ground = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Ground"));
	Ground->SetupAttachment(Origin);
	Ground->SetMobility(EComponentMobility::Movable);
	Ground->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Ground->SetRelativeLocation(FVector(0.0, 0.0, -25.0));
	Ground->SetRelativeScale3D(FVector(4000.0, 4000.0, 1.0));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Plane(TEXT("/Engine/BasicShapes/Plane.Plane"));
	Ground->SetStaticMesh(Plane.Object);
}

void AVsimWorld::BeginPlay()
{
	Super::BeginPlay();
	Reader = MakeUnique<FVsimLinkReader>(LinkName);

	// PBR materials made in the editor by Content/Python/init_unreal.py; the
	// engine's basic shape material (colour only) if they are not there.
	PbrMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Vsim/M_VsimPBR.M_VsimPBR"));
	PbrTwoSidedMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Vsim/M_VsimPBR_TwoSided.M_VsimPBR_TwoSided"));
	PaintMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Vsim/M_VsimPaint.M_VsimPaint"));
	BasicMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (!PbrMaterial)
	{
		UE_LOG(LogTemp, Warning, TEXT("vsim: /Game/Vsim/M_VsimPBR missing, using basic materials (open the project in the editor once)"));
	}
	if (UMaterialInterface* GrassParent = PbrMaterial ? PbrMaterial.Get() : BasicMaterial.Get())
	{
		UMaterialInstanceDynamic* Grass = UMaterialInstanceDynamic::Create(GrassParent, this);
		Grass->SetVectorParameterValue(TEXT("Color"), FLinearColor::FromSRGBColor(FColor(78, 104, 58)));
		Grass->SetScalarParameterValue(TEXT("Roughness"), 1.0f);
		Ground->SetMaterial(0, Grass);
	}

	FActorSpawnParameters Spawn;
	Spawn.Owner = this;
	Spawn.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	UWorld* World = GetWorld();
	Track = World->SpawnActor<AVsimTrackActor>(AVsimTrackActor::StaticClass(), FTransform::Identity, Spawn);
	Car = World->SpawnActor<AVsimCarActor>(AVsimCarActor::StaticClass(), FTransform::Identity, Spawn);
	FreeCamera = World->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), FTransform::Identity, Spawn);
	Car->UseCamera(Camera);
}

void AVsimWorld::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Reader)
	{
		Reader->SendKeys(0);
	}
	Reader.Reset();
	Super::EndPlay(EndPlayReason);
}

FString AVsimWorld::GetTrackName() const
{
	return Track ? Track->TrackName() : FString();
}

APlayerController* AVsimWorld::Player() const
{
	return GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
}

void AVsimWorld::SendKeys(uint32 Keys)
{
	if (Reader && bConnected)
	{
		Reader->SendKeys(Keys);
	}
}

void AVsimWorld::SendCommand(vsim_link::Command Command)
{
	if (Reader && bConnected)
	{
		Reader->SendCommand(Command);
	}
}

void AVsimWorld::CycleCamera()
{
	do
	{
		Camera = static_cast<EVsimCamera>((static_cast<uint8>(Camera) + 1) % static_cast<uint8>(EVsimCamera::Count));
	} while (Camera == EVsimCamera::Tv && TvSpots.Num() == 0);
	Car->UseCamera(Camera);
}

bool AVsimWorld::ToggleSound()
{
	return Car && Car->Sound->Toggle();
}

bool AVsimWorld::IsSoundOn() const
{
	return Car && Car->Sound->IsPlaying();
}

void AVsimWorld::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	FrameMs = 1000.0 * DeltaSeconds;
	bConnected = Reader && Reader->Connect();
	if (bConnected)
	{
		vsim_link::VehicleState Newest;
		if (Reader->Read(Newest))
		{
			State = Newest;
		}
		UpdateTrack();
		UpdateCar();
		Car->ConfigureCameras(State.info);
		Car->ApplyState(State);
		// How old the newest driver input in this state is when the game
		// thread uses it; the frame being built reaches the screen about one
		// frame later, so that is added for the estimate shown and reported.
		InputLatencyMs = State.input_ns != 0 ? double(vsim_link::now_ns() - State.input_ns) / 1e6 : 0.0;
		Reader->ReportViewer(InputLatencyMs + FrameMs, FrameMs);
	}
	else if (IsSoundOn())
	{
		// No simulator: fade to silence instead of holding the last sound.
		vsim_link::VehicleState Silent{};
		Silent.paused = 1;
		Car->Sound->SetState(Silent);
	}
	UpdateCamera(DeltaSeconds);
}

FVsimMaterialSet AVsimWorld::MaterialSet() const
{
	FVsimMaterialSet Set;
	Set.Pbr = PbrMaterial;
	Set.PbrTwoSided = PbrTwoSidedMaterial;
	Set.Paint = PaintMaterial;
	Set.Basic = BasicMaterial;
	return Set;
}

void AVsimWorld::UpdateTrack()
{
	uint64 Generation = 0;
	const vsim_link::MeshRegion* Region = Reader->GetTrack(Generation);
	if (Generation == TrackGeneration)
	{
		return;
	}
	if (Generation == 0)
	{
		Track->Clear();
		TvSpots.Reset();
		TrackGeneration = 0;
	}
	else if (Region)
	{
		Track->Build(*Region, MaterialSet(), bFlipWinding);
		TvSpots = Track->CameraSpots();
		TrackGeneration = Generation;
	}
	// else: not mappable yet, try again next frame
}

void AVsimWorld::UpdateCar()
{
	uint64 Generation = 0;
	const vsim_link::MeshRegion* Region = Reader->GetCar(Generation);
	if (Generation == CarGeneration || Generation == 0 || !Region)
	{
		return;
	}
	Car->Build(*Region, MaterialSet(), bFlipWinding);
	CarGeneration = Generation;
}

void AVsimWorld::UpdateCamera(float DeltaSeconds)
{
	APlayerController* PC = Player();
	if (!PC || !Car)
	{
		return;
	}
	const FVector CarPos = Car->GetActorLocation();
	AActor* Target = Car;
	if (Camera == EVsimCamera::Tv && TvSpots.Num() > 0)
	{
		const FVector* Best = &TvSpots[0];
		for (const FVector& Spot : TvSpots)
		{
			if (FVector::DistSquared(Spot, CarPos) < FVector::DistSquared(*Best, CarPos))
			{
				Best = &Spot;
			}
		}
		const double Distance = FVector::Dist(*Best, CarPos) / 100.0;
		FreeCamera->SetActorLocationAndRotation(*Best, (CarPos + FVector(0, 0, 40) - *Best).Rotation());
		// Zoom so the car keeps roughly the same size.
		const float Fov = float(FMath::Clamp(FMath::RadiansToDegrees(2.0 * FMath::Atan(5.0 / FMath::Max(Distance, 1.0))), 6.0, 60.0));
		FreeCamera->GetCameraComponent()->SetFieldOfView(Fov);
		Target = FreeCamera;
	}
	else if (Camera == EVsimCamera::Top)
	{
		FreeCamera->SetActorLocationAndRotation(CarPos + FVector(0, 0, 5500), FRotator(-90.0, 0.0, 0.0));
		FreeCamera->GetCameraComponent()->SetFieldOfView(50.0f);
		Target = FreeCamera;
	}
	if (PC->GetViewTarget() != Target)
	{
		PC->SetViewTarget(Target);
	}
}
