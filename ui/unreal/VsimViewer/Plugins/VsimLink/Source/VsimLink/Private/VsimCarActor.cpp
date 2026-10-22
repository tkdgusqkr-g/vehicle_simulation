#include "VsimCarActor.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/SpringArmComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	const FLinearColor BodyRed = FLinearColor::FromSRGBColor(FColor(215, 38, 61));
	const FLinearColor Carbon = FLinearColor::FromSRGBColor(FColor(27, 30, 35));
	const FLinearColor White = FLinearColor::FromSRGBColor(FColor(242, 244, 247));
	const FLinearColor Tyre = FLinearColor::FromSRGBColor(FColor(17, 18, 20));
	const FLinearColor Rim = FLinearColor::FromSRGBColor(FColor(184, 190, 199));
	const FLinearColor HelmetYellow = FLinearColor::FromSRGBColor(FColor(255, 210, 63));
	const FLinearColor Metal = FLinearColor::FromSRGBColor(FColor(138, 145, 155));

	// Car frame in metres (x forward, y left, z up) to Unreal centimetres.
	FVector Cm(const FVector& M) { return FVector(100.0 * M.X, -100.0 * M.Y, 100.0 * M.Z); }
}

AVsimCarActor::AVsimCarActor()
{
	PrimaryActorTick.bCanEverTick = false;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Root->SetMobility(EComponentMobility::Movable);
	RootComponent = Root;
	Body = CreateDefaultSubobject<USceneComponent>(TEXT("Body"));
	Body->SetupAttachment(Root);

	ChaseArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("ChaseArm"));
	ChaseArm->SetupAttachment(Root);
	ChaseArm->SetRelativeLocation(FVector(0.0, 0.0, 60.0));
	ChaseArm->SetRelativeRotation(FRotator(-11.0, 0.0, 0.0));
	ChaseArm->TargetArmLength = 520.0f;
	ChaseArm->bDoCollisionTest = false;
	ChaseArm->bEnableCameraLag = true;
	ChaseArm->CameraLagSpeed = 12.0f;
	ChaseArm->bEnableCameraRotationLag = true;
	ChaseArm->CameraRotationLagSpeed = 6.0f;
	ChaseArm->bInheritPitch = false;
	ChaseArm->bInheritRoll = false;
	ChaseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
	ChaseCamera->SetupAttachment(ChaseArm, USpringArmComponent::SocketName);
	ChaseCamera->SetFieldOfView(70.0f);

	CockpitCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("CockpitCamera"));
	CockpitCamera->SetupAttachment(Body);
	CockpitCamera->SetFieldOfView(80.0f);
	CockpitCamera->SetAutoActivate(false);
	NoseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("NoseCamera"));
	NoseCamera->SetupAttachment(Body);
	NoseCamera->SetFieldOfView(75.0f);
	NoseCamera->SetAutoActivate(false);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(
		TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	Cube = CubeMesh.Object;
	Cylinder = CylinderMesh.Object;
	Sphere = SphereMesh.Object;
	BaseMaterial = Material.Object;
}

UStaticMeshComponent* AVsimCarActor::AddPart(USceneComponent* Parent, UStaticMesh* Shape, const FVector& CentreM,
	const FVector& ScaleM, const FLinearColor& Color, const FRotator& Rotation)
{
	// The basic shapes are 1 m across, so the scale is the size in metres.
	UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
	Part->SetStaticMesh(Shape);
	Part->SetMobility(EComponentMobility::Movable);
	Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Part->SetupAttachment(Parent);
	Part->SetRelativeLocation(Cm(CentreM));
	Part->SetRelativeRotation(Rotation);
	Part->SetRelativeScale3D(ScaleM);
	Part->RegisterComponent();
	UMaterialInstanceDynamic* Paint = UMaterialInstanceDynamic::Create(BaseMaterial, this);
	Paint->SetVectorParameterValue(TEXT("Color"), Color);
	Part->SetMaterial(0, Paint);
	Parts.Add(Part);
	return Part;
}

USceneComponent* AVsimCarActor::AddPivot(USceneComponent* Parent, const FVector& LocationM)
{
	USceneComponent* Pivot = NewObject<USceneComponent>(this);
	Pivot->SetMobility(EComponentMobility::Movable);
	Pivot->SetupAttachment(Parent);
	Pivot->SetRelativeLocation(Cm(LocationM));
	Pivot->RegisterComponent();
	Parts.Add(Pivot);
	return Pivot;
}

void AVsimCarActor::ClearParts()
{
	for (USceneComponent* Part : Parts)
	{
		if (Part)
		{
			Part->DestroyComponent();
		}
	}
	Parts.Reset();
	WheelSteer.Reset();
	WheelSpin.Reset();
	SteeringWheel = nullptr;
	Helmet = nullptr;
}

void AVsimCarActor::Configure(const vsim_link::VehicleInfo& Info)
{
	const float Dimensions[7] = {Info.wheelbase, Info.cg_to_front, Info.cg_height, Info.track_front,
		Info.track_rear, Info.tire_radius, Info.tire_width};
	if (Parts.Num() > 0 && FMemory::Memcmp(Dimensions, Built, sizeof(Built)) == 0)
	{
		return;
	}
	FMemory::Memcpy(Built, Dimensions, sizeof(Built));
	ClearParts();

	// Sensible sizes until the simulator has sent its vehicle.
	const double L = Info.wheelbase > 0.5f ? Info.wheelbase : 1.53;
	const double A = Info.cg_to_front > 0.1f ? Info.cg_to_front : 0.8;
	const double B = L - A;
	const double H = Info.cg_height > 0.05f ? Info.cg_height : 0.28;
	const double TrackFront = Info.track_front > 0.3f ? Info.track_front : 1.22;
	const double TrackRear = Info.track_rear > 0.3f ? Info.track_rear : 1.18;
	const double R = Info.tire_radius > 0.05f ? Info.tire_radius : 0.203;
	const double W = Info.tire_width > 0.02f ? Info.tire_width : 0.19;
	CgHeight = H;

	// Body parts hang from Body, which pivots at the CG; positions below are
	// from the ground, so subtract the CG height.
	Body->SetRelativeLocation(FVector(0.0, 0.0, 100.0 * H));
	const auto At = [H](double X, double Y, double Z) { return FVector(X, Y, Z - H); };
	const double TubLength = A + 0.55 * B;
	AddPart(Body, Cube, At(A - TubLength / 2 - 0.05, 0, 0.27), FVector(TubLength, 0.56, 0.30), BodyRed);
	AddPart(Body, Cube, At(A + 0.30, 0, 0.23), FVector(0.70, 0.36, 0.20), BodyRed);
	AddPart(Body, Cube, At(A + 0.70, 0, 0.18), FVector(0.22, 0.22, 0.12), BodyRed);
	AddPart(Body, Cube, At(-0.15 * B, 0.40, 0.22), FVector(0.75, 0.26, 0.24), BodyRed);
	AddPart(Body, Cube, At(-0.15 * B, -0.40, 0.22), FVector(0.75, 0.26, 0.24), BodyRed);
	AddPart(Body, Cube, At(A - 0.55, 0, 0.43), FVector(0.80, 0.57, 0.04), White);
	AddPart(Body, Cube, At(-0.05, 0, 0.44), FVector(0.55, 0.42, 0.08), Carbon);
	AddPart(Body, Cube, At(-0.55 * B, 0, 0.32), FVector(0.75 * B, 0.50, 0.38), Carbon);
	// Roll hoop and driver.
	AddPart(Body, Cube, At(-0.36, 0.20, 0.62), FVector(0.05, 0.05, 0.55), Metal);
	AddPart(Body, Cube, At(-0.36, -0.20, 0.62), FVector(0.05, 0.05, 0.55), Metal);
	AddPart(Body, Cube, At(-0.36, 0, 0.89), FVector(0.05, 0.45, 0.05), Metal);
	Helmet = AddPart(Body, Sphere, At(-0.18, 0, 0.74), FVector(0.26), HelmetYellow);
	// Front wing with endplates, rear wing on two pylons.
	const double FrontSpan = TrackFront + 0.15;
	AddPart(Body, Cube, At(A + 0.62, 0, 0.07), FVector(0.30, FrontSpan, 0.025), Carbon);
	AddPart(Body, Cube, At(A + 0.58, 0, 0.13), FVector(0.22, FrontSpan - 0.1, 0.02), Carbon);
	AddPart(Body, Cube, At(A + 0.62, FrontSpan / 2, 0.11), FVector(0.36, 0.015, 0.14), Carbon);
	AddPart(Body, Cube, At(A + 0.62, -FrontSpan / 2, 0.11), FVector(0.36, 0.015, 0.14), Carbon);
	const double RearSpan = FMath::Min(TrackRear, 1.1);
	AddPart(Body, Cube, At(-B - 0.32, 0, 1.02), FVector(0.32, RearSpan, 0.025), Carbon);
	AddPart(Body, Cube, At(-B - 0.40, 0, 1.12), FVector(0.22, RearSpan, 0.02), Carbon);
	AddPart(Body, Cube, At(-B - 0.34, RearSpan / 2, 1.02), FVector(0.45, 0.015, 0.32), Carbon);
	AddPart(Body, Cube, At(-B - 0.34, -RearSpan / 2, 1.02), FVector(0.45, 0.015, 0.32), Carbon);
	AddPart(Body, Cube, At(-B - 0.25, 0.15, 0.70), FVector(0.05, 0.03, 0.62), Carbon);
	AddPart(Body, Cube, At(-B - 0.25, -0.15, 0.70), FVector(0.05, 0.03, 0.62), Carbon);

	// Steering wheel: the column tilts, the wheel turns about the column (local X).
	USceneComponent* Column = AddPivot(Body, At(0.18, 0, 0.55));
	Column->SetRelativeRotation(FRotator(-12.0, 0.0, 0.0));
	SteeringWheel = AddPivot(Column, FVector::ZeroVector);
	AddPart(SteeringWheel, Cylinder, FVector::ZeroVector, FVector(0.26, 0.26, 0.02), Carbon, FRotator(90.0, 0.0, 0.0));
	AddPart(SteeringWheel, Cube, FVector(-0.012, 0, 0.09), FVector(0.01, 0.03, 0.06), White);

	// Cameras: driver's eye and nose.
	CockpitCamera->SetRelativeLocation(Cm(At(-0.12, 0, 0.86)));
	CockpitCamera->SetRelativeRotation(FRotator(-5.0, 0.0, 0.0));
	NoseCamera->SetRelativeLocation(Cm(At(A + 0.15, 0, 0.48)));
	NoseCamera->SetRelativeRotation(FRotator(-2.0, 0.0, 0.0));

	// Wheels on the root (no body roll), in vsim corner order FL, FR, RL, RR.
	const FVector Corners[4] = {FVector(A, TrackFront / 2, R), FVector(A, -TrackFront / 2, R),
		FVector(-B, TrackRear / 2, R), FVector(-B, -TrackRear / 2, R)};
	for (const FVector& Corner : Corners)
	{
		USceneComponent* Steer = AddPivot(Root, Corner);
		USceneComponent* Spin = AddPivot(Steer, FVector::ZeroVector);
		// The cylinder's axis is Z; roll it onto the axle (Y).
		AddPart(Spin, Cylinder, FVector::ZeroVector, FVector(2 * R, 2 * R, W), Tyre, FRotator(0.0, 0.0, 90.0));
		AddPart(Spin, Cylinder, FVector::ZeroVector, FVector(1.25 * R, 1.25 * R, W + 0.012), Rim, FRotator(0.0, 0.0, 90.0));
		AddPart(Spin, Cube, FVector::ZeroVector, FVector(1.15 * R, W + 0.02, 0.035), White);  // shows the rotation
		WheelSteer.Add(Steer);
		WheelSpin.Add(Spin);
		// Wishbones from the tub to the upright.
		const double Side = Corner.Y > 0 ? 1.0 : -1.0;
		for (const double Dz : {-0.06, 0.07})
		{
			const FVector Inner(Corner.X, Side * 0.25, R + Dz);
			const FVector Outer(Corner.X, Corner.Y - Side * W / 2, R + Dz);
			const FVector Mid = (Inner + Outer) / 2;
			const double Length = FVector::Dist(Inner, Outer);
			AddPart(Root, Cylinder, Mid, FVector(0.025, 0.025, Length), Metal, FRotator(0.0, 0.0, 90.0));
		}
	}
}

void AVsimCarActor::ApplyState(const vsim_link::VehicleState& State)
{
	SetActorLocationAndRotation(VsimFrame::Position(State.x, State.y, 0.0), VsimFrame::Heading(State.yaw));
	// Unreal: positive pitch lifts the nose, positive roll lowers the right
	// side. vsim: positive pitch dips the nose, positive roll lifts the left.
	Body->SetRelativeRotation(FRotator(-VsimFrame::Degrees(State.pitch), 0.0, VsimFrame::Degrees(State.roll)));
	for (int32 Corner = 0; Corner < WheelSteer.Num() && Corner < 4; ++Corner)
	{
		WheelSteer[Corner]->SetRelativeRotation(FRotator(0.0, -VsimFrame::Degrees(State.wheel_steer[Corner]), 0.0));
		// Rolling forward turns the top of the wheel forward: negative pitch.
		WheelSpin[Corner]->SetRelativeRotation(FRotator(-VsimFrame::Degrees(State.wheel_spin[Corner]), 0.0, 0.0));
	}
	if (SteeringWheel)
	{
		// Turning left is counter-clockwise from the seat: negative roll.
		SteeringWheel->SetRelativeRotation(FRotator(0.0, 0.0, -VsimFrame::Degrees(State.steering_wheel)));
	}
}

void AVsimCarActor::UseCamera(EVsimCamera Camera)
{
	ChaseCamera->SetActive(Camera == EVsimCamera::Chase);
	CockpitCamera->SetActive(Camera == EVsimCamera::Cockpit);
	NoseCamera->SetActive(Camera == EVsimCamera::Nose);
	if (Helmet)
	{
		Helmet->SetVisibility(Camera != EVsimCamera::Cockpit);
	}
}
