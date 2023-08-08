// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleInterface.h"

class VEHICLEPARTS_API FVehiclePartsModule : public IModuleInterface
{
public:

	virtual void StartupModule() override {};
	virtual void ShutdownModule() override {};
};
