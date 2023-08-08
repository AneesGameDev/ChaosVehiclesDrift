// Copyright Epic Games, Inc. All Rights Reserved.

namespace UnrealBuildTool.Rules
{
    public class VehicleParts : ModuleRules
    {
        public VehicleParts(ReadOnlyTargetRules target) : base(target)
        {

            PublicDependencyModuleNames.AddRange(
                new [] {
                "Core",
                "CoreUObject",
				"Chaos", 
                "Engine",
                $"PhysicsControl"
                }
            ); 

			PrivateDefinitions.Add("CHAOS_INCLUDE_LEVEL_1=1");
		}
    }
}
