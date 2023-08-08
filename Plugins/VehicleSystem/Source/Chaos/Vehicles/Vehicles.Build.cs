// Copyright Epic Games, Inc. All Rights Reserved.

namespace UnrealBuildTool.Rules
{
	public class Vehicles : ModuleRules
	{

        public Vehicles(ReadOnlyTargetRules target) : base(target)
		{

			PublicDependencyModuleNames.AddRange(
				new []
				{
					"Core",
					"CoreUObject",
                    "Engine",
                    "EngineSettings",
                    "RenderCore",
                    "AnimGraphRuntime",
                    "RHI",
                    nameof(VehicleParts),
				}
				);

            SetupModulePhysicsSupport(target);
			PrivateDefinitions.Add("CHAOS_INCLUDE_LEVEL_1=1");
		}
	}
}
