// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"
#include "Modules/ModuleManager.h"
#include "PhysicsInterfaceDeclaresCore.h"

/**
 * The public interface to this module
 */
class IVehiclesModule : public IModuleInterface
{

	static inline IVehiclesModule& Get()
	{
		return FModuleManager::LoadModuleChecked< IVehiclesModule >( "Vehicles" );
	}

	/**
	 * Checks to see if this module is loaded and ready.  It is only valid to call Get() if IsAvailable() returns true.
	 *
	 * @return True if the module is loaded and ready to use
	 */
	static inline bool IsAvailable()
	{
		return FModuleManager::Get().IsModuleLoaded( "Vehicles" );
	}

};

