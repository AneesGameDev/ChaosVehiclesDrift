// Copyright Epic Games, Inc. All Rights Reserved.


#include "WheeledVehicles.h"

//#include "AssetToolsModule.h"
#include "CoreMinimal.h"
#include "HAL/ConsoleManager.h"
#include "Modules/ModuleManager.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ChaosVehicleManager.h"
#include "..\Public\ChaosWheeledVehicleManagerAsyncCallback.h"


IMPLEMENT_MODULE(FWheeledVehiclesModule, WheeledVehicles);


void FWheeledVehiclesModule::PhysSceneInit(FPhysScene* PhysScene) {
	FChaosVehicleManager* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene);
	if (VehicleManager == nullptr)
	{
		VehicleManager = new FChaosVehicleManager(PhysScene);
		
		/*VehicleManager->DetachFromPhysScene(PhysScene);
		delete VehicleManager;
		VehicleManager = nullptr;*/
	}
	auto batch = new FWheeledVehiclesBatch(UChaosWheeledVehicleMovementComponent::StaticClass()->GetName());
	VehicleManager->RegisterBatch(batch);
}

void FWheeledVehiclesModule::PhysSceneTerm(FPhysScene* PhysScene)
{
	FChaosVehicleManager* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene);
	if (VehicleManager != nullptr)
	{
		VehicleManager->DetachFromPhysScene(PhysScene);
		delete VehicleManager;
		VehicleManager = nullptr;
	}
}

void FWheeledVehiclesModule::StartupModule()
{
	OnPhysSceneInitHandle = FPhysicsDelegates::OnPhysSceneInit.AddRaw(this, &FWheeledVehiclesModule::PhysSceneInit);
	OnPhysSceneTermHandle = FPhysicsDelegates::OnPhysSceneTerm.AddRaw(this, &FWheeledVehiclesModule::PhysSceneTerm);
}


void FWheeledVehiclesModule::ShutdownModule()
{
	FPhysicsDelegates::OnPhysSceneInit.Remove(OnPhysSceneInitHandle);
	FPhysicsDelegates::OnPhysSceneTerm.Remove(OnPhysSceneTermHandle);
}

