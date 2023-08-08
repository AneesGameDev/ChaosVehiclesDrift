// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Containers/Array.h"
#include "Math/UnrealMathSSE.h"
#include "Math/Vector.h"
#include "TireSystem.h"
#include "VehicleSystemTemplate.h"
#include "VehicleUtility.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif


namespace Chaos
{

	struct VEHICLEPARTS_API FSimpleHoverConfig
	{
		FSimpleHoverConfig():
			MaxSpeed(0),
			ForwardTorque(0),
			ReverseTorque(0),
			Deceleration(0),
			EquilibriumOffset(0),
			SideMomentumForce(0),
			Steering(0) {
		}

		float MaxSpeed;
		float ForwardTorque;
		float ReverseTorque;
		float Deceleration;
		float EquilibriumOffset;
		float SideMomentumForce;
		float Steering;
	};

/**
* Wheel instance data changes during the simulation
*/
class VEHICLEPARTS_API FSimpleHoverSim : public TVehicleSystem<FSimpleHoverConfig>
{
public:

	FSimpleHoverSim(const FSimpleHoverConfig* SetupIn);

};



} // namespace Chaos

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
