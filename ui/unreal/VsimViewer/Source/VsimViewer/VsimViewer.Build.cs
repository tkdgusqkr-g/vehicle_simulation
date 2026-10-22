using UnrealBuildTool;

public class VsimViewer : ModuleRules
{
	public VsimViewer(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "VsimLink" });
	}
}
