// Copyright Epic Games, Inc. All Rights Reserved.

namespace UnrealBuildTool.Rules
{
	public class HoveringVehicles : ModuleRules
	{

        public HoveringVehicles(ReadOnlyTargetRules Target) : base(Target)
		{
			PublicIncludePaths.Add(ModuleDirectory + "/Public");

			PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
			PublicDependencyModuleNames.AddRange(
				new string[]
				{
					"Core",
					"CoreUObject",
                    "Engine",
                    "EngineSettings",
                    "RenderCore",
                    "AnimGraphRuntime",
                    "RHI",
                    "PhysicsCore",
                    "Chaos",
                    nameof(Vehicles),
                    nameof(VehicleParts),
				}
				);
		}
	}
}
