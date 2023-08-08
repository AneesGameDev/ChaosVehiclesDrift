// Copyright Epic Games, Inc. All Rights Reserved.

#include "HoverSystem.h"

#include "WheelSystem.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

namespace Chaos
{
	FSimpleHoverSim::FSimpleHoverSim(const FSimpleHoverConfig* SetupIn) : TVehicleSystem<FSimpleHoverConfig>(SetupIn)
	{
		
	}
} // namespace Chaos

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
