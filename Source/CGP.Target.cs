// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class CGPTarget : TargetRules
{
	public CGPTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		DefaultBuildSettings = BuildSettingsVersion.V2;

		ExtraModuleNames.AddRange( new [] { "CGP","VehicleSystem" } );
	}
}
