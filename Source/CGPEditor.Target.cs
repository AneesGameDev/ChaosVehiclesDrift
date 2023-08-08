// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class CGPEditorTarget : TargetRules
{
	public CGPEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		DefaultBuildSettings = BuildSettingsVersion.V2;

		ExtraModuleNames.AddRange( new [] { "CGP" } );
	}
}
