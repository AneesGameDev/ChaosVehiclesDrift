// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "PhysicsPublic.h"

#include "Chaos/SimCallbackInput.h"
#include "Chaos/SimCallbackObject.h"
#include "Chaos/GeometryParticlesfwd.h"

DECLARE_STATS_GROUP(TEXT("ChaosVehicleManager"), STATGROUP_ChaosVehicleManager, STATGROUP_Advanced);
class UChaosTireConfig;
class UChaosVehicleMovementComponent;
class FChaosScene;

enum EChaosAsyncVehicleDataType : int8
{
	AsyncInvalid,
	AsyncDefault,
};


class VEHICLES_API FVehiclesBatch {
public:
	FString id;
	FVehiclesBatch(FString _id) : id(_id)
	{
		
	}
	virtual ~FVehiclesBatch() = default;
	virtual void RegisterCallbacks(const FPhysScene_Chaos& scene) = 0;
	virtual void UnregisterCallbacks(const FPhysScene_Chaos& scene) = 0;
	virtual void AddVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> vehicle) = 0;
	virtual bool RemoveVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> vehicle) = 0;
	virtual void Update(float DeltaTime) = 0;
	virtual void ParallelUpdate(int32& Timestamp, FPhysScene_Chaos& scene, float DeltaSeconds) = 0;
	virtual void PreTick(float DeltaTime) = 0;
	virtual int32 PostUpdate() = 0;
	virtual int VehiclesCount() = 0;
};

class VEHICLES_API FChaosVehicleManager
{
public:
	// Updated when vehicles need to recreate their physics state.
	// Used when values tweaked while the game is running.
	static uint32 VehicleSetupTag;

	FChaosVehicleManager(FPhysScene* PhysScene);
	~FChaosVehicleManager();

	/** Get Physics Scene */
	FPhysScene_Chaos& GetScene() const { return Scene; }

	/**
	 * Register a Physics vehicle for processing
	 */
	void AddVehicle(const FString& id, TWeakObjectPtr<UChaosVehicleMovementComponent> Vehicle);
	void RemoveVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> Vehicle);

	/**
	 * Update vehicle tuning and other state such as input
	 */
	void ScenePreTick(FPhysScene* PhysScene, float DeltaTime);

	/** Detach this vehicle manager from a FPhysScene (remove delegates, remove from map etc) */
	void DetachFromPhysScene(FPhysScene* PhysScene);

	void Update(FPhysScene* PhysScene, float DeltaTime);
	void PostUpdate(FChaosScene* PhysScene);
	int GetRegisteredVehiclesCount();

	void ParallelUpdateVehicles(float DeltaSeconds);
	void RegisterBatch(FVehiclesBatch* batch);
	void UnregisterBatch(const FString& id);

	/** Find a vehicle manager from an FPhysScene */
	static FChaosVehicleManager* GetVehicleManagerFromScene(FPhysScene* PhysScene);

	void RegisterCallbacks();
	void UnregisterCallbacks();

private:
	/** Map of physics scenes to corresponding vehicle manager */
	static TMap<FPhysScene*, FChaosVehicleManager*> SceneToVehicleManagerMap;

	// The physics scene we belong to
	FPhysScene_Chaos& Scene;

	static bool GInitialized;

	// All instanced vehicles

	FDelegateHandle OnPhysScenePreTickHandle;
	FDelegateHandle OnPhysScenePostTickHandle;

	static FDelegateHandle OnPostWorldInitializationHandle;
	static FDelegateHandle OnWorldCleanupHandle;


	int32 Timestamp;
	int32 SubStepCount;

	TArray<FVehiclesBatch*> batches;
};

