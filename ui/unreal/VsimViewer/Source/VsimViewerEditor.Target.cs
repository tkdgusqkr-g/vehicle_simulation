using UnrealBuildTool;

public class VsimViewerEditorTarget : TargetRules
{
	public VsimViewerEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("VsimViewer");
	}
}
