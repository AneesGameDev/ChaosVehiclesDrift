// Copyright Epic Games, Inc. All Rights Reserved.


#include "VehiclesEditorModule.h"

#include "AssetToolsModule.h"
#include "CoreMinimal.h"
#include "ChaosVehiclesEditorStyle.h"
#include "ChaosVehiclesEditorCommands.h"
#include "HAL/ConsoleManager.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
//#include "VehiclesEditorDetails.h"
#include "ChaosVehicleManager.h"

IMPLEMENT_MODULE( IVehiclesEditorModule, VehiclesEditor )



void IVehiclesEditorModule::StartupModule()
{
	//OnUpdatePhysXMaterialHandle = FPhysicsDelegates::OnUpdatePhysXMaterial.AddRaw(this, &FPhysXVehiclesPlugin::UpdatePhysXMaterial);
	//OnPhysicsAssetChangedHandle = FPhysicsDelegates::OnPhysicsAssetChanged.AddRaw(this, &FPhysXVehiclesPlugin::PhysicsAssetChanged);

	FVehiclesEditorStyle::Get();

	FAssetToolsModule& AssetToolsModule = FAssetToolsModule::GetModule();
	IAssetTools& AssetTools = AssetToolsModule.Get();

	if (GIsEditor && !IsRunningCommandlet())
	{
	}

	// Register details view customizations
	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
}


void IVehiclesEditorModule::ShutdownModule()
{
	//FPhysicsDelegates::OnUpdatePhysXMaterial.Remove(OnUpdatePhysXMaterialHandle);
	//FPhysicsDelegates::OnPhysicsAssetChanged.Remove(OnPhysicsAssetChangedHandle);

	if (UObjectInitialized())
	{
		FAssetToolsModule& AssetToolsModule = FAssetToolsModule::GetModule();
		IAssetTools& AssetTools = AssetToolsModule.Get();
	}

	// Unregister details view customizations
	FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.UnregisterCustomPropertyTypeLayout("ChaosDebugSubstepControl");
}
