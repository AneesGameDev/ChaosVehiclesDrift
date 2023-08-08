// Copyright Epic Games, Inc. All Rights Reserved.

namespace UnrealBuildTool.Rules
{
	public class VehiclesEditor : ModuleRules
	{
        public VehiclesEditor(ReadOnlyTargetRules Target) : base(Target)
		{
			PublicDependencyModuleNames.AddRange(
				new string[]
				{
					"Core",
					"CoreUObject",
					"Slate",
					"SlateCore",
					"Engine",
					"EditorFramework",
					"UnrealEd",
					"PropertyEditor",
					"AnimGraphRuntime",
					"AnimGraph",
					"BlueprintGraph",
					"ToolMenus",
					"PhysicsCore",
					nameof(VehicleParts),
					nameof(Vehicles),
					nameof(WheeledVehicles)
				}
				);

			PrivateDefinitions.Add("CHAOS_INCLUDE_LEVEL_1=1");
		}
	}
}
