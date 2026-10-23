#include "VsimCarActor.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"

AVsimCarActor::AVsimCarActor()
{
	PrimaryActorTick.bCanEverTick = false;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Root->SetMobility(EComponentMobility::Movable);
	RootComponent = Root;
	Body = CreateDefaultSubobject<USceneComponent>(TEXT("Body"));
	Body->SetupAttachment(Root);
	Body->SetRelativeLocation(FVector(0.0, 0.0, 28.0));

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
}

USceneComponent* AVsimCarActor::NewPivot(USceneComponent* Parent, const FVector& Location)
{
	USceneComponent* Pivot = NewObject<USceneComponent>(this);
	Pivot->SetMobility(EComponentMobility::Movable);
	Pivot->SetupAttachment(Parent);
	Pivot->SetRelativeLocation(Location);
	Pivot->RegisterComponent();
	Parts.Add(Pivot);
	return Pivot;
}

UProceduralMeshComponent* AVsimCarActor::NewMesh(USceneComponent* Parent, const FVector& Location)
{
	UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this);
	Mesh->SetMobility(EComponentMobility::Movable);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Mesh->SetCastShadow(true);
	Mesh->SetupAttachment(Parent);
	Mesh->SetRelativeLocation(Location);
	Mesh->RegisterComponent();
	Parts.Add(Mesh);
	return Mesh;
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
	Materials.Reset();
	Links.Reset();
	SteeringWheels.Reset();
	for (FCorner& Corner : Corners)
	{
		Corner = FCorner();
	}
}

void AVsimCarActor::Build(const vsim_link::MeshRegion& Region, const FVsimMaterialSet& MaterialSet, bool bFlipWinding)
{
	ClearParts();
	CgHeight = Region.cg_height > 0.05f ? Region.cg_height : 0.28;
	Body->SetRelativeLocation(FVector(0.0, 0.0, 100.0 * CgHeight));
	const FVector CgOrigin(0.0, 0.0, CgHeight);  // body vertices are relative to the CG pivot

	const vsim_link::MeshPart* PartTable = vsim_link::mesh_parts(Region);
	TArray<UMaterialInstanceDynamic*> Created;
	// Steer and spin pivots first, so uprights and links can find their hub.
	for (uint32 Index = 0; Index < Region.part_count; ++Index)
	{
		const vsim_link::MeshPart& Part = PartTable[Index];
		if (Part.role == vsim_link::kRoleWheel && Part.corner >= 0 && Part.corner < 4)
		{
			FCorner& Corner = Corners[Part.corner];
			Corner.Hub = VsimFrame::Position(Part.pivot[0], Part.pivot[1], Part.pivot[2]);
			Corner.Steer = NewPivot(Root, Corner.Hub);
			Corner.Spin = NewPivot(Corner.Steer, FVector::ZeroVector);
		}
	}
	for (uint32 Index = 0; Index < Region.part_count; ++Index)
	{
		const vsim_link::MeshPart& Part = PartTable[Index];
		const bool bCorner = Part.corner >= 0 && Part.corner < 4 && Corners[Part.corner].Steer != nullptr;
		UProceduralMeshComponent* Mesh = nullptr;
		FVector Origin = FVector::ZeroVector;
		switch (Part.role)
		{
		case vsim_link::kRoleWheel:
			Mesh = bCorner ? NewMesh(Corners[Part.corner].Spin, FVector::ZeroVector) : nullptr;
			break;
		case vsim_link::kRoleUpright:
			Mesh = bCorner ? NewMesh(Corners[Part.corner].Steer, FVector::ZeroVector) : nullptr;
			break;
		case vsim_link::kRoleSteeringWheel:
		{
			FSteeringWheel Wheel;
			const FVector Pivot = VsimFrame::Position(Part.pivot[0], Part.pivot[1], Part.pivot[2] - CgHeight);
			Wheel.Pivot = NewPivot(Body, Pivot);
			Wheel.Axis = VsimFrame::Direction(Part.axis[0], Part.axis[1], Part.axis[2]).GetSafeNormal();
			Mesh = NewMesh(Wheel.Pivot, FVector::ZeroVector);
			SteeringWheels.Add(Wheel);
			break;
		}
		case vsim_link::kRoleLink:
		{
			if (!bCorner)
			{
				break;
			}
			FLink Link;
			Link.Mesh = NewMesh(Root, FVector::ZeroVector);
			Link.Inboard = VsimFrame::Position(Part.pivot[0], Part.pivot[1], Part.pivot[2]);
			Link.Outer = VsimFrame::Position(Part.outer[0], Part.outer[1], Part.outer[2]);
			Link.Corner = Part.corner;
			Link.bSteers = (Part.flags & vsim_link::kPartSteers) != 0;
			Mesh = Link.Mesh;
			Links.Add(Link);
			break;
		}
		default:  // body and anything unknown ride on the body
			Mesh = NewMesh(Body, FVector::ZeroVector);
			Origin = CgOrigin;
			break;
		}
		if (!Mesh)
		{
			continue;
		}
		VsimMesh::AddSections(Mesh, Region, Part.first_section, Part.section_count, 0, Origin, bFlipWinding,
			MaterialSet, this, Created);
	}
	for (UMaterialInstanceDynamic* Material : Created)
	{
		Materials.Add(Material);
	}
}

void AVsimCarActor::ConfigureCameras(const vsim_link::VehicleInfo& Info)
{
	if (Info.cg_to_front == ConfiguredFor)
	{
		return;
	}
	ConfiguredFor = Info.cg_to_front;
	const double A = Info.cg_to_front > 0.1f ? Info.cg_to_front : 0.8;
	// Driver's eye of a formula student car, and the nose camera, relative to the CG pivot.
	CockpitCamera->SetRelativeLocation(VsimFrame::Position(A - 0.88, 0.0, 0.95 - CgHeight));
	CockpitCamera->SetRelativeRotation(FRotator(-5.0, 0.0, 0.0));
	NoseCamera->SetRelativeLocation(VsimFrame::Position(A + 0.15, 0.0, 0.50 - CgHeight));
	NoseCamera->SetRelativeRotation(FRotator(-2.0, 0.0, 0.0));
}

void AVsimCarActor::ApplyState(const vsim_link::VehicleState& State)
{
	SetActorLocationAndRotation(VsimFrame::Position(State.x, State.y, State.road_z), VsimFrame::Heading(State.yaw));
	// Unreal: positive pitch lifts the nose, positive roll lowers the right
	// side. vsim: positive pitch dips the nose, positive roll lifts the left.
	const double BodyHeight = State.z > 0.05 ? State.z : CgHeight;
	Body->SetRelativeLocation(FVector(0.0, 0.0, 100.0 * BodyHeight));
	const FRotator BodyRotation(-VsimFrame::Degrees(State.pitch), 0.0, VsimFrame::Degrees(State.roll));
	Body->SetRelativeRotation(BodyRotation);

	for (int32 Index = 0; Index < 4; ++Index)
	{
		FCorner& Corner = Corners[Index];
		if (!Corner.Steer)
		{
			continue;
		}
		Corner.Steer->SetRelativeLocation(Corner.Hub + FVector(0.0, 0.0, 100.0 * State.wheel_z[Index]));
		// Steering left (vsim +) is negative yaw in Unreal.
		Corner.Steer->SetRelativeRotation(FRotator(0.0, -VsimFrame::Degrees(State.wheel_steer[Index]), 0.0));
		// Rolling forward is a positive rotation about Unreal's +Y.
		Corner.Spin->SetRelativeRotation(FQuat(FVector(0.0, 1.0, 0.0), State.wheel_spin[Index]));
	}
	for (FSteeringWheel& Wheel : SteeringWheels)
	{
		// The y mirror turns a right-handed angle into a left-handed one.
		Wheel.Pivot->SetRelativeRotation(FQuat(Wheel.Axis, -State.steering_wheel));
	}
	// Links: inboard end on the body (pitched and rolled about the CG),
	// outboard end on the hub (moved by travel, turned by steering).
	const FVector BodyOrigin(0.0, 0.0, 100.0 * BodyHeight);
	const FVector CgPivot(0.0, 0.0, 100.0 * CgHeight);
	for (FLink& Link : Links)
	{
		const FVector Inboard = BodyOrigin + BodyRotation.RotateVector(Link.Inboard - CgPivot);
		const FCorner& Corner = Corners[Link.Corner];
		const FVector Hub = Corner.Hub + FVector(0.0, 0.0, 100.0 * State.wheel_z[Link.Corner]);
		const FVector Outer = Link.bSteers
			? FRotator(0.0, -VsimFrame::Degrees(State.wheel_steer[Link.Corner]), 0.0).RotateVector(Link.Outer)
			: Link.Outer;
		const FVector Span = Hub + Outer - Inboard;
		const double Length = Span.Size();
		if (Length < 1.0)
		{
			continue;
		}
		Link.Mesh->SetRelativeLocation(Inboard);
		Link.Mesh->SetRelativeRotation(FRotationMatrix::MakeFromX(Span).Rotator());
		Link.Mesh->SetRelativeScale3D(FVector(Length / 100.0, 1.0, 1.0));  // the tube is 1 m long
	}
}

void AVsimCarActor::UseCamera(EVsimCamera Camera)
{
	ChaseCamera->SetActive(Camera == EVsimCamera::Chase);
	CockpitCamera->SetActive(Camera == EVsimCamera::Cockpit);
	NoseCamera->SetActive(Camera == EVsimCamera::Nose);
	// The cockpit camera sits inside the helmet, whose one-sided surfaces are
	// invisible from inside: nothing to hide.
}
