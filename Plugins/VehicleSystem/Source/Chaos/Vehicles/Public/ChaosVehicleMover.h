// Copyright Epic Games, Inc. All Rights Reserved.

/*
 * Component to handle the vehicle simulation for an actor
 */

#pragma once

#include "CoreMinimal.h"
#include "ChaosVehicleMover.generated.h"

	UENUM()
	enum class ESweepShape : uint8
	{
		/** Use ray to determine suspension length to ground - fastest */
		Raycast		UMETA(DisplayName = "Raycast"),

		/** Use sphere to determine suspension length to ground */
		Spherecast	UMETA(DisplayName = "Spherecast"),

		/** Use wheel collision shape to determine suspension length to ground - Slowest */
		Shapecast	UMETA(DisplayName = "Shapecast")
	};

	UENUM()
	enum class ESweepType : uint8
	{
		/** Sweeps against simple geometry only */
		SimpleSweep				UMETA(DisplayName = "SimpleSweep"),
		/** Sweeps against complex geometry only */
		ComplexSweep			UMETA(DisplayName = "ComplexSweep")
	};


UENUM()
enum class EAxleType : uint8
{
	Undefined = 0,
	Front,
	Rear
};

struct VEHICLES_API FWheelTraceParams
{
	ESweepType SweepType;
	ESweepShape SweepShape;
};
	UENUM()
	enum class ETorqueCombineMethod : uint8
	{
		/** External torque value has no effect - default **/
		None = 0,
		/** completely replace existing torques, can set and forget will apply same torque every frame until zeroed */
		Override,
		/** Combine external torque with existing torques, must set external torque every frame */
		Additive
	};

	UCLASS(Abstract,BlueprintType, Blueprintable)
		class VEHICLES_API UChaosVehicleMover : public UObject
	{
		GENERATED_BODY()

	};

