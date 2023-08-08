// Copyright Epic Games, Inc. All Rights Reserved.

#include "ChaosVehicleManager.h"
#include "UObject/UObjectIterator.h"

#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Physics/PhysicsFiltering.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "ChaosVehicleMovementComponent.h"

#include "PBDRigidsSolver.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

DECLARE_CYCLE_STAT(TEXT("VehicleManager:ParallelUpdateVehicles"), STAT_ChaosVehicleManager_ParallelUpdateVehicles,
                   STATGROUP_ChaosVehicleManager);
DECLARE_CYCLE_STAT(TEXT("VehicleManager:Update"), STAT_ChaosVehicleManager_Update, STATGROUP_ChaosVehicleManager);
DECLARE_CYCLE_STAT(TEXT("VehicleManager:ScenePreTick"), STAT_ChaosVehicleManager_ScenePreTick,
                   STATGROUP_ChaosVehicleManager);

DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("NumVehiclesTotal"), STAT_NumVehicles_Dynamic, STATGROUP_ChaosVehicleManager);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("NumVehiclesAwake"), STAT_NumVehicles_Awake, STATGROUP_ChaosVehicleManager);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("NumVehiclesSleeping"), STAT_NumVehicles_Sleeping, STATGROUP_ChaosVehicleManager);

extern FVehicleDebugParams GVehicleDebugParams;

TMap<FPhysScene*, FChaosVehicleManager*> FChaosVehicleManager::SceneToVehicleManagerMap;
uint32 FChaosVehicleManager::VehicleSetupTag = 0;

FDelegateHandle FChaosVehicleManager::OnPostWorldInitializationHandle;
FDelegateHandle FChaosVehicleManager::OnWorldCleanupHandle;

bool FChaosVehicleManager::GInitialized = false;

void OnPostWorldInitialize(UWorld* InWorld, const UWorld::InitializationValues) {
	if (auto Manager = FChaosVehicleManager::GetVehicleManagerFromScene(InWorld->GetPhysicsScene())) {
		Manager->RegisterCallbacks();
	}
}

void OnWorldCleanup(UWorld* InWorld, bool bSessionEnded, bool bCleanupResources) {
	if (auto Manager = FChaosVehicleManager::GetVehicleManagerFromScene(InWorld->GetPhysicsScene())) {
		Manager->UnregisterCallbacks();
	}
}

FChaosVehicleManager::FChaosVehicleManager(FPhysScene* PhysScene)
	: Scene(*PhysScene)
	  , Timestamp(0)
	  , batches(TArray<FVehiclesBatch*>()) {
	check(PhysScene);

	if (!GInitialized) {
		GInitialized = true;
		// PhysScene->GetOwningWorld() is always null here, the world is being setup too late to be of use
		// therefore setup these global world delegates that will callback when everything is setup so registering
		// the physics solver Async Callback will succeed
		OnPostWorldInitializationHandle = FWorldDelegates::OnPostWorldInitialization.AddStatic(&OnPostWorldInitialize);
		OnWorldCleanupHandle = FWorldDelegates::OnWorldCleanup.AddStatic(&OnWorldCleanup);
	}

	ensure(FChaosVehicleManager::SceneToVehicleManagerMap.Find(PhysScene) == nullptr);
	//double registration with same scene, will cause a leak

	// Add to Scene-To-Manager map
	SceneToVehicleManagerMap.Add(PhysScene, this);
}


void FChaosVehicleManager::RegisterCallbacks() {
	OnPhysScenePreTickHandle = Scene.OnPhysScenePreTick.AddRaw(this, &FChaosVehicleManager::Update);
	OnPhysScenePostTickHandle = Scene.OnPhysScenePostTick.AddRaw(this, &FChaosVehicleManager::PostUpdate);

	for (auto& batch : batches) {
		batch->RegisterCallbacks(Scene);
	}
}

void FChaosVehicleManager::UnregisterCallbacks() {
	Scene.OnPhysScenePreTick.Remove(OnPhysScenePreTickHandle);
	Scene.OnPhysScenePostTick.Remove(OnPhysScenePostTickHandle);
	for (auto& batch : batches) {
		batch->UnregisterCallbacks(Scene);
	}
}

void FChaosVehicleManager::DetachFromPhysScene(FPhysScene* PhysScene) {
	UnregisterCallbacks();

	if (auto world = PhysScene->GetOwningWorld()) {
		world->OnWorldBeginPlay.RemoveAll(this);
	}

	SceneToVehicleManagerMap.Remove(PhysScene);
}

FChaosVehicleManager::~FChaosVehicleManager() {
	for (FVehiclesBatch* batch : batches) {
		delete batch;
	}
}

FChaosVehicleManager* FChaosVehicleManager::GetVehicleManagerFromScene(FPhysScene* PhysScene) {
	FChaosVehicleManager* Manager = nullptr;
	FChaosVehicleManager** ManagerPtr = SceneToVehicleManagerMap.Find(PhysScene);
	if (ManagerPtr != nullptr) {
		Manager = *ManagerPtr;
	}
	return Manager;
}

void FChaosVehicleManager::AddVehicle(const FString& id, TWeakObjectPtr<UChaosVehicleMovementComponent> Vehicle) {
	check(Vehicle != NULL);
	check(Vehicle->HasValidPhysicsState());


	if (auto batch = batches.FindByPredicate([id](const auto _batch) {
		return _batch->id == id;
	})) {
		(*batch)->AddVehicle(Vehicle);
	}
}

void FChaosVehicleManager::RemoveVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> Vehicle) {
	check(Vehicle != NULL);
	check(Vehicle->HasValidPhysicsState());

	for (auto& batch : batches) {
		if (batch->RemoveVehicle(Vehicle)) {
			break;
		}
	}
}

void FChaosVehicleManager::ScenePreTick(FPhysScene* PhysScene, float DeltaTime) {
	// inputs being set via back door, i.e. accessing PVehicle directly is a no go now, needs to go through async input system
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicleManager_ScenePreTick);

	for (auto& batch : batches) {
		batch->PreTick(DeltaTime);
	}
}

void FChaosVehicleManager::Update(FPhysScene* PhysScene, float DeltaTime) {
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicleManager_Update);

	UWorld* World = Scene.GetOwningWorld();

	SubStepCount = 0;

	ScenePreTick(PhysScene, DeltaTime);

	ParallelUpdateVehicles(DeltaTime);

	if (World) {
		for (auto& batch : batches) {
			batch->Update(DeltaTime);
		}
	}
}

void FChaosVehicleManager::PostUpdate(FChaosScene* PhysScene) {
	SET_DWORD_STAT(STAT_NumVehicles_Dynamic, GetRegisteredVehiclesCount());

	int32 SleepingCount = 0;
	for (auto& batch : batches) {
		SleepingCount += batch->PostUpdate();
	}
	SET_DWORD_STAT(STAT_NumVehicles_Awake, GetRegisteredVehiclesCount() - SleepingCount);
	SET_DWORD_STAT(STAT_NumVehicles_Sleeping, SleepingCount);
}

int FChaosVehicleManager::GetRegisteredVehiclesCount() {
	int res = 0;
	for (auto& batch : batches) {
		res += batch->VehiclesCount();
	}
	return res;
}

void FChaosVehicleManager::ParallelUpdateVehicles(float DeltaSeconds) {
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicleManager_ParallelUpdateVehicles);
	for (auto& batch : batches) {
		batch->ParallelUpdate(Timestamp, Scene, DeltaSeconds);
	}
}

void FChaosVehicleManager::RegisterBatch(FVehiclesBatch* batch) {
	if (!batches.ContainsByPredicate([&batch](const auto _batch) {
		return _batch->id == batch->id;
	})) {
		batches.Emplace(batch);
	}
}

void FChaosVehicleManager::UnregisterBatch(const FString& id) {
	auto batch = batches.FindByPredicate([id](const auto _batch) {
		return _batch->id == id;
	});
	if (*batch) {
		batches.Remove(*batch);
	}
}
