using UnrealBuildTool;

public class VsimLink : ModuleRules
{
	public VsimLink(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		// Keeps the Windows API (shared memory) apart from the other files.
		bUseUnity = false;
		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine", "InputCore", "ProceduralMeshComponent", "AudioMixer"
		});
	}
}
