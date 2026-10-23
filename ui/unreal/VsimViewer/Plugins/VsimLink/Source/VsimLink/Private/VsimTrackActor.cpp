#include "VsimTrackActor.h"

#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "VsimTypes.h"

AVsimTrackActor::AVsimTrackActor()
{
	PrimaryActorTick.bCanEverTick = false;
	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
	RootComponent = Mesh;
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Mesh->SetCastShadow(true);
}

void AVsimTrackActor::Clear()
{
	Mesh->ClearAllMeshSections();
	Materials.Reset();
	RightEdge.Reset();
	LeftEdge.Reset();
	Name.Reset();
}

void AVsimTrackActor::Build(const vsim_link::MeshRegion& Region, const FVsimMaterialSet& MaterialSet, bool bFlipWinding)
{
	Clear();
	ANSICHAR TrackName[sizeof(Region.name) + 1] = {};
	FMemory::Memcpy(TrackName, Region.name, sizeof(Region.name));
	Name = UTF8_TO_TCHAR(TrackName);

	TArray<UMaterialInstanceDynamic*> Created;
	VsimMesh::AddSections(Mesh, Region, 0, Region.section_count, 0, FVector::ZeroVector, bFlipWinding, MaterialSet,
		this, Created);
	for (UMaterialInstanceDynamic* Material : Created)
	{
		Materials.Add(Material);
	}

	// Road edges for the TV cameras: the asphalt rows, right then left.
	const vsim_link::MeshSectionInfo* Sections = vsim_link::mesh_sections(Region);
	for (uint32 Index = 0; Index < Region.section_count; ++Index)
	{
		const vsim_link::MeshSectionInfo& Section = Sections[Index];
		if (VsimMesh::SectionName(Section) != TEXT("asphalt") || !vsim_link::section_valid(Region, Section))
		{
			continue;
		}
		const float* P = vsim_link::mesh_array<float>(Region, Section.positions);
		for (uint32 Row = 0; Row + 1 < Section.vertex_count; Row += 2)
		{
			RightEdge.Add(VsimFrame::Position(P[3 * Row], P[3 * Row + 1], P[3 * Row + 2]));
			LeftEdge.Add(VsimFrame::Position(P[3 * Row + 3], P[3 * Row + 4], P[3 * Row + 5]));
		}
	}
}

TArray<FVector> AVsimTrackActor::CameraSpots(double Spacing, double Offset, double Height) const
{
	TArray<FVector> Spots;
	double Travelled = 0.0;
	for (int32 Row = 1; Row < RightEdge.Num(); ++Row)
	{
		Travelled += FVector::Dist(RightEdge[Row], RightEdge[Row - 1]) / 100.0;
		if (Travelled < Spacing)
		{
			continue;
		}
		Travelled = 0.0;
		const FVector Outward = (RightEdge[Row] - LeftEdge[Row]).GetSafeNormal2D();
		Spots.Add(RightEdge[Row] + Outward * Offset * 100.0 + FVector(0.0, 0.0, Height * 100.0));
	}
	return Spots;
}
