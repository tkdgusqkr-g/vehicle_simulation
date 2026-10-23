#include "VsimMesh.h"

#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "VsimTypes.h"

FString VsimMesh::SectionName(const vsim_link::MeshSectionInfo& Section)
{
	ANSICHAR Name[sizeof(Section.name) + 1] = {};
	FMemory::Memcpy(Name, Section.name, sizeof(Section.name));
	return FString(UTF8_TO_TCHAR(Name));
}

int32 VsimMesh::AddSections(UProceduralMeshComponent* Mesh, const vsim_link::MeshRegion& Region, uint32 First,
	uint32 Count, int32 SectionBase, const FVector& Origin, bool bFlipWinding, const FVsimMaterialSet& Set,
	UObject* Outer, TArray<UMaterialInstanceDynamic*>& OutMaterials)
{
	const vsim_link::MeshSectionInfo* Sections = vsim_link::mesh_sections(Region);
	int32 Added = 0;
	for (uint32 Index = First; Index < First + Count && Index < Region.section_count; ++Index)
	{
		const vsim_link::MeshSectionInfo& Section = Sections[Index];
		if (!vsim_link::section_valid(Region, Section))
		{
			continue;
		}
		const float* P = vsim_link::mesh_array<float>(Region, Section.positions);
		const float* N = vsim_link::mesh_array<float>(Region, Section.normals);
		const float* UV = vsim_link::mesh_array<float>(Region, Section.uvs);
		const uint32* I = vsim_link::mesh_array<uint32>(Region, Section.indices);

		TArray<FVector> Positions;
		TArray<FVector> Normals;
		TArray<FVector2D> UV0;
		Positions.Reserve(Section.vertex_count);
		Normals.Reserve(Section.vertex_count);
		UV0.Reserve(Section.vertex_count);
		for (uint32 V = 0; V < Section.vertex_count; ++V)
		{
			Positions.Add(VsimFrame::Position(P[3 * V] - Origin.X, P[3 * V + 1] - Origin.Y, P[3 * V + 2] - Origin.Z));
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
		const int32 Target = SectionBase + Added;
		Mesh->CreateMeshSection_LinearColor(Target, Positions, Triangles, Normals, UV0, TArray<FLinearColor>(),
			TArray<FProcMeshTangent>(), false);

		const FString Name = SectionName(Section);
		const bool bTwoSided = (Section.flags & vsim_link::kSectionDoubleSided) != 0;
		UMaterialInterface* Parent = Set.Basic;
		if (Set.Pbr)
		{
			Parent = bTwoSided && Set.PbrTwoSided ? Set.PbrTwoSided : Set.Pbr;
			if (Name.StartsWith(TEXT("paint")) && Set.Paint)
			{
				Parent = Set.Paint;
			}
		}
		if (Parent)
		{
			UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Parent, Outer);
			const FColor Srgb = FLinearColor(Section.color[0], Section.color[1], Section.color[2]).ToFColor(false);
			Material->SetVectorParameterValue(TEXT("Color"), FLinearColor::FromSRGBColor(Srgb));
			Material->SetScalarParameterValue(TEXT("Metallic"), Section.metallic);
			Material->SetScalarParameterValue(TEXT("Roughness"), Section.roughness);
			Mesh->SetMaterial(Target, Material);
			OutMaterials.Add(Material);
		}
		++Added;
	}
	return Added;
}
