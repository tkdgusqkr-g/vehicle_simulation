using UnrealBuildTool;

public class VsimViewerTarget : TargetRules
{
	public VsimViewerTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("VsimViewer");
	}
}
