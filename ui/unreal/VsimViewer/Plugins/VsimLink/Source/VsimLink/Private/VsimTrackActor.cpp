#include "VsimTrackActor.h"

#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "VsimTypes.h"

AVsimTrackActor::AVsimTrackActor()
{
	PrimaryActorTick.bCanEverTick = false;
	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
	RootComponent = Mesh;
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Mesh->bUseAsyncCooking = true;
	Mesh->SetCastShadow(true);

	// Engine material with a "Color" vector parameter.
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(
		TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	BaseMaterial = Material.Object;
}

void AVsimTrackActor::Clear()
{
	Mesh->ClearAllMeshSections();
	Materials.Reset();
	RightEdge.Reset();
	LeftEdge.Reset();
	Name.Reset();
}

bool AVsimTrackActor::Build(const vsim_link::TrackRegion& Region, bool bFlipWinding)
{
	Clear();
	const uint64 HeaderBytes = sizeof(vsim_link::TrackRegion) + uint64(Region.section_count) * sizeof(vsim_link::TrackSection);
	if (HeaderBytes > Region.size)
	{
		return false;
	}
	ANSICHAR TrackName[sizeof(Region.name) + 1] = {};
	FMemory::Memcpy(TrackName, Region.name, sizeof(Region.name));
	Name = UTF8_TO_TCHAR(TrackName);

	const vsim_link::TrackSection* Sections = vsim_link::track_sections(Region);
	for (uint32 Index = 0; Index < Region.section_count; ++Index)
	{
		const vsim_link::TrackSection& Section = Sections[Index];
		const uint64 Vertices = Section.vertex_count;
		const auto Fits = [&](uint64 Offset, uint64 Bytes) { return Offset + Bytes <= Region.size; };
		if (!Fits(Section.positions, 12 * Vertices) || !Fits(Section.normals, 12 * Vertices) ||
			!Fits(Section.uvs, 8 * Vertices) || !Fits(Section.indices, 4 * uint64(Section.index_count)))
		{
			Clear();
			return false;
		}
		const float* P = vsim_link::track_array<float>(Region, Section.positions);
		const float* N = vsim_link::track_array<float>(Region, Section.normals);
		const float* UV = vsim_link::track_array<float>(Region, Section.uvs);
		const uint32* I = vsim_link::track_array<uint32>(Region, Section.indices);

		TArray<FVector> Positions;
		TArray<FVector> Normals;
		TArray<FVector2D> UV0;
		Positions.Reserve(Section.vertex_count);
		Normals.Reserve(Section.vertex_count);
		UV0.Reserve(Section.vertex_count);
		for (uint32 V = 0; V < Section.vertex_count; ++V)
		{
			Positions.Add(VsimFrame::Position(P[3 * V], P[3 * V + 1], P[3 * V + 2]));
			Normals.Add(VsimFrame::Direction(N[3 * V], N[3 * V + 1], N[3 * V + 2]));
			UV0.Add(FVector2D(UV[2 * V], UV[2 * V + 1]));
		}
		// Counter-clockwise in vsim's right-handed frame is what Unreal draws
		// as the front face after the y mirror; bFlipWinding is the escape hatch.
		TArray<int32> Triangles;
		Triangles.Reserve(Section.index_count);
		for (uint32 T = 0; T + 2 < Section.index_count; T += 3)
		{
			const int32 A = int32(I[T]), B = int32(I[T + 1]), C = int32(I[T + 2]);
			if (uint32(FMath::Max3(A, B, C)) >= Section.vertex_count)
			{
				continue;
			}
			Triangles.Add(A);
			Triangles.Add(bFlipWinding ? C : B);
			Triangles.Add(bFlipWinding ? B : C);
		}
		Mesh->CreateMeshSection_LinearColor(int32(Index), Positions, Triangles, Normals, UV0, TArray<FLinearColor>(),
			TArray<FProcMeshTangent>(), false);

		UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(BaseMaterial, this);
		const FColor Srgb = FLinearColor(Section.color[0], Section.color[1], Section.color[2]).ToFColor(false);
		Material->SetVectorParameterValue(TEXT("Color"), FLinearColor::FromSRGBColor(Srgb));
		Mesh->SetMaterial(int32(Index), Material);
		Materials.Add(Material);

		ANSICHAR SectionName[sizeof(Section.name) + 1] = {};
		FMemory::Memcpy(SectionName, Section.name, sizeof(Section.name));
		if (FCStringAnsi::Strcmp(SectionName, "asphalt") == 0)
		{
			for (int32 Row = 0; Row + 1 < Positions.Num(); Row += 2)
			{
				RightEdge.Add(Positions[Row]);
				LeftEdge.Add(Positions[Row + 1]);
			}
		}
	}
	return true;
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
