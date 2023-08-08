// Copyright Epic Games, Inc. All Rights Reserved.


#include "HoveringVehicles.h"

//#include "AssetToolsModule.h"
#include "CoreMinimal.h"
#include "HAL/ConsoleManager.h"
#include "Modules/ModuleManager.h"
#include "ChaosHoveringVehicleMovementComponent.h"
#include "ChaosVehicleManager.h"
#include "..\Public\ChaosHoveringVehicleManagerAsyncCallback.h"


IMPLEMENT_MODULE(FHoveringVehiclesModule, HoveringVehicles);


void FHoveringVehiclesModule::PhysSceneInit(FPhysScene* PhysScene) {
	FChaosVehicleManager* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene);
	if (VehicleManager == nullptr)
	{
		VehicleManager = new FChaosVehicleManager(PhysScene);
		
		/*VehicleManager->DetachFromPhysScene(PhysScene);
		delete VehicleManager;
		VehicleManager = nullptr;*/
	}
	auto batch = new FHoveringVehiclesBatch(UChaosHoveringVehicleMovementComponent::StaticClass()->GetName());
	VehicleManager->RegisterBatch(batch);
}

void FHoveringVehiclesModule::PhysSceneTerm(FPhysScene* PhysScene)
{
	FChaosVehicleManager* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene);
	if (VehicleManager != nullptr)
	{
		VehicleManager->DetachFromPhysScene(PhysScene);
		delete VehicleManager;
		VehicleManager = nullptr;
	}
}

void FHoveringVehiclesModule::StartupModule()
{
	OnPhysSceneInitHandle = FPhysicsDelegates::OnPhysSceneInit.AddRaw(this, &FHoveringVehiclesModule::PhysSceneInit);
	OnPhysSceneTermHandle = FPhysicsDelegates::OnPhysSceneTerm.AddRaw(this, &FHoveringVehiclesModule::PhysSceneTerm);
}


void FHoveringVehiclesModule::ShutdownModule()
{
	FPhysicsDelegates::OnPhysSceneInit.Remove(OnPhysSceneInitHandle);
	FPhysicsDelegates::OnPhysSceneTerm.Remove(OnPhysSceneTermHandle);
}

