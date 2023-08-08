// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "ChaosVehicleManager.h"
#include "ChaosHoveringVehicleMovementComponent.h"
#include "ChaosVehicleMovementComponent.h"
#include "ChaosVehicleMover.h"
#include "PhysicsPublic.h"
#include "Chaos/SimCallbackInput.h"
#include "Chaos/SimCallbackObject.h"
#include "Chaos/GeometryParticlesfwd.h"
#include "PhysicsProxy/SingleParticlePhysicsProxyFwd.h"

struct HOVERINGVEHICLES_API FElevatorOutput
{
	FElevatorOutput()
		: InContact(false)
		, AngularPosition(0.f)
		, AngularVelocity(0.f)
		, WheelRadius(0.f)
		, SuspensionOffset(0.f)
		, SpringForce(0.f)
		, NormalizedSuspensionLength(0.f)
		, bBlockingHit(false)
		, ImpactPoint(FVector::ZeroVector)
		, PhysMaterial(nullptr)
	{
	}

	// wheels
	bool InContact;
	float AngularPosition;
	float AngularVelocity;
	float WheelRadius;


	// suspension related
	float SuspensionOffset;
	float SpringForce;
	float NormalizedSuspensionLength;

	bool bBlockingHit;
	FVector ImpactPoint;
	TWeakObjectPtr<UPhysicalMaterial> PhysMaterial;
};

/**
 * Per Vehicle Output State from Physics Thread to Game Thread                            
 */
struct HOVERINGVEHICLES_API FHoveringVehicleOutput
{
	FHoveringVehicleOutput()
		: CurrentGear(0)
		, TargetGear(0)
		, EngineRPM(0.f)
		, EngineTorque(0.f)
		, TransmissionRPM(0.f)
		, TransmissionTorque(0.f)
	{
	}

	TArray<FElevatorOutput> elevators;
	int32 CurrentGear;
	int32 TargetGear;
	float EngineRPM;
	float EngineTorque;
	float TransmissionRPM;
	float TransmissionTorque;
};

/**
 * Per Vehicle Input State from Game Thread to Physics Thread
 */                             
struct HOVERINGVEHICLES_API FChaosVehicleAsyncInput
{
	const EChaosAsyncVehicleDataType Type;
	class UChaosHoveringVehicleMovementComponent* Vehicle;
	
	Chaos::FSingleParticlePhysicsProxy* Proxy;

	/** 
	* Vehicle simulation running on the Physics Thread
	*/
	virtual TUniquePtr<struct FHoveringVehicleAsyncOutput> Simulate(UWorld* World, const float DeltaSeconds, const float TotalSeconds, bool& bWakeOut) const = 0;

	virtual void ApplyDeferredForces(Chaos::FRigidBodyHandle_Internal* RigidHandle) const = 0;

	FChaosVehicleAsyncInput(EChaosAsyncVehicleDataType InType = EChaosAsyncVehicleDataType::AsyncInvalid)
		: Type(InType)
		, Vehicle(nullptr)
	{
		Proxy = nullptr;	//indicates async/sync task not needed
	}

	virtual ~FChaosVehicleAsyncInput() = default;
};

struct FChaosVehicleManagerAsyncInput : public Chaos::FSimCallbackInput
{
	TArray<TUniquePtr<FChaosVehicleAsyncInput>> VehicleInputs;

	TWeakObjectPtr<UWorld> World;
	int32 Timestamp = INDEX_NONE;

	void Reset()
	{
		VehicleInputs.Reset();
		World.Reset();
	}
};

/**
 * Async Output Data
 */
struct HOVERINGVEHICLES_API FHoveringVehicleAsyncOutput
{
	const EChaosAsyncVehicleDataType Type;
	bool bValid;	// indicates no work was done
	FHoveringVehicleOutput VehicleSimOutput;

	FHoveringVehicleAsyncOutput(EChaosAsyncVehicleDataType InType = EChaosAsyncVehicleDataType::AsyncInvalid)
		: Type(InType)
		, bValid(false)
	{ }

	virtual ~FHoveringVehicleAsyncOutput() = default;
};


/**
 * Async Output for all of the vehicles handled by this Vehicle Manager
 */
struct FChaosVehicleManagerAsyncOutput : public Chaos::FSimCallbackOutput
{
	TArray<TUniquePtr<FHoveringVehicleAsyncOutput>> VehicleOutputs;
	int32 Timestamp = INDEX_NONE;

	void Reset()
	{
		VehicleOutputs.Reset();
	}
};

struct HOVERINGVEHICLES_API FHoveringVehicleAsyncInput : public FChaosVehicleAsyncInput
{
	float GravityZ;
	FControlInputs ControlInputs;
	mutable FCollisionQueryParams TraceParams;
	mutable FCollisionResponseContainer TraceCollisionResponse;
	mutable TArray<FWheelTraceParams> WheelTraceParams;

	FHoveringVehicleAsyncInput();

	virtual TUniquePtr<FHoveringVehicleAsyncOutput> Simulate(UWorld* World, const float DeltaSeconds, const float TotalSeconds, bool& bWakeOut) const override;
	
	virtual void ApplyDeferredForces(Chaos::FRigidBodyHandle_Internal* RigidHandle) const override;

};

/**
 * Async callback from the Physics Engine where we can perform our vehicle simulation
 */
class HOVERINGVEHICLES_API FChaosHoveringVehicleManagerAsyncCallback : public Chaos::TSimCallbackObject<FChaosVehicleManagerAsyncInput, FChaosVehicleManagerAsyncOutput, Chaos::ESimCallbackOptions::Presimulate | Chaos::ESimCallbackOptions::ContactModification>
{
	virtual void OnPreSimulate_Internal() override;
	virtual void OnContactModification_Internal(Chaos::FCollisionContactModifier& Modifications) override;
};

class HOVERINGVEHICLES_API FHoveringVehiclesBatch : public FVehiclesBatch {
public:
	FHoveringVehiclesBatch(FString _id) :FVehiclesBatch(_id), AsyncCallback(nullptr) {
		
	}
	virtual ~FHoveringVehiclesBatch() override;

	virtual void RegisterCallbacks(const FPhysScene_Chaos& scene) override;

	virtual void UnregisterCallbacks(const FPhysScene_Chaos& scene) override;

	virtual void AddVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> vehicle) override;
	virtual bool RemoveVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> vehicle) override;
	
virtual int32 PostUpdate() override;
	virtual void Update(float DeltaTime) override;
virtual void PreTick(float DeltaTime) override;
	virtual void ParallelUpdate(int32& Timestamp, FPhysScene_Chaos& scene, float DeltaSeconds) override;
virtual int VehiclesCount() override {return Vehicles.Num();}
private:
	TArray<TWeakObjectPtr<UChaosHoveringVehicleMovementComponent>> Vehicles;
	void AddVehicle_Internal(TWeakObjectPtr<UChaosHoveringVehicleMovementComponent> vehicle);
	bool RemoveVehicle_Internal(TWeakObjectPtr<UChaosHoveringVehicleMovementComponent> vehicle);
	FChaosHoveringVehicleManagerAsyncCallback* AsyncCallback;	// Async callback from the physics engine - we can run our simulation here

	TArray<Chaos::TSimCallbackOutputHandle<FChaosVehicleManagerAsyncOutput>> PendingOutputs;
	Chaos::TSimCallbackOutputHandle<FChaosVehicleManagerAsyncOutput> LatestOutput;
};
