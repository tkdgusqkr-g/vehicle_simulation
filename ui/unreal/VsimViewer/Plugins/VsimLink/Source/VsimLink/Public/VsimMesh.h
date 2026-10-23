// Turns mesh sections from the link (vsim_link.h) into procedural mesh
// sections with PBR materials. Shared by the track and the car.
#pragma once

#include "CoreMinimal.h"
#include "vsim_link.h"

class UMaterialInstanceDynamic;
class UMaterialInterface;
class UObject;
class UProceduralMeshComponent;

/** Materials to instance per section; any of them may be missing. */
struct FVsimMaterialSet
{
	/** /Game/Vsim/M_VsimPBR: Color, Metallic, Roughness (Content/Python/init_unreal.py makes it). */
	UMaterialInterface* Pbr = nullptr;
	UMaterialInterface* PbrTwoSided = nullptr;
	/** /Game/Vsim/M_VsimPaint: clear coat car paint, same parameters. */
	UMaterialInterface* Paint = nullptr;
	/** Engine BasicShapeMaterial (Color only), the fallback. */
	UMaterialInterface* Basic = nullptr;
};

namespace VsimMesh
{
	/**
	 * Adds sections [First, First + Count) of the region to Mesh, starting at
	 * mesh section SectionBase, with vertices moved by -Origin (metres, vsim
	 * frame). Returns the number of sections added; the materials created
	 * are appended to OutMaterials (keep them referenced).
	 */
	VSIMLINK_API int32 AddSections(UProceduralMeshComponent* Mesh, const vsim_link::MeshRegion& Region, uint32 First,
		uint32 Count, int32 SectionBase, const FVector& Origin, bool bFlipWinding, const FVsimMaterialSet& Set,
		UObject* Outer, TArray<UMaterialInstanceDynamic*>& OutMaterials);

	/** The name of a section as a string. */
	VSIMLINK_API FString SectionName(const vsim_link::MeshSectionInfo& Section);
}
