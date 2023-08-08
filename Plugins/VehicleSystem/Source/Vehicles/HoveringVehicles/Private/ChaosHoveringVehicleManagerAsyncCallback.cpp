// Copyright Epic Games, Inc. All Rights Reserved.

#include "..\Public\ChaosHoveringVehicleManagerAsyncCallback.h"
#include "ChaosVehicleManager.h"
#include "ChaosVehicleMovementComponent.h"
#include "PBDRigidsSolver.h"
#include "Chaos/ParticleHandleFwd.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

extern FVehicleDebugParams GVehicleDebugParams;

DECLARE_CYCLE_STAT(TEXT("AsyncCallback:OnPreSimulate_Internal"), STAT_AsyncCallback_OnPreSimulate, STATGROUP_ChaosVehicleManager);

/**
 * Callback from Physics thread
 */
void FChaosHoveringVehicleManagerAsyncCallback::OnPreSimulate_Internal()
{
	using namespace Chaos;

	SCOPE_CYCLE_COUNTER(STAT_AsyncCallback_OnPreSimulate);

	float DeltaTime = GetDeltaTime_Internal();
	float SimTime = GetSimTime_Internal();

	const FChaosVehicleManagerAsyncInput* Input = GetConsumerInput_Internal();
	if (Input == nullptr)
	{
		return;
	}

	const int32 NumVehicles = Input->VehicleInputs.Num();

	UWorld* World = Input->World.Get();	//only safe to access for scene queries
	if (World == nullptr || NumVehicles == 0)
	{
		//world is gone so don't bother, or nothing to simulate.
		return;
	}

	Chaos::FPhysicsSolver* PhysicsSolver = static_cast<Chaos::FPhysicsSolver*>(GetSolver());
	if (PhysicsSolver == nullptr)
	{
		return;
	}

	FChaosVehicleManagerAsyncOutput& Output = GetProducerOutputData_Internal();
	Output.VehicleOutputs.AddDefaulted(NumVehicles);
	Output.Timestamp = Input->Timestamp;

	const TArray<TUniquePtr<FChaosVehicleAsyncInput>>& InputVehiclesBatch = Input->VehicleInputs;
	TArray<TUniquePtr<FHoveringVehicleAsyncOutput>>& OutputVehiclesBatch = Output.VehicleOutputs;

	// beware running the vehicle simulation in parallel, code must remain threadsafe
	auto LambdaParallelUpdate = [World, DeltaTime, SimTime, &InputVehiclesBatch, &OutputVehiclesBatch](int32 Idx)
	{
		const FChaosVehicleAsyncInput& VehicleInput = *InputVehiclesBatch[Idx];

		if (VehicleInput.Proxy == nullptr || VehicleInput.Proxy->GetPhysicsThreadAPI() == nullptr)
		{
			return;
		}

		Chaos::FRigidBodyHandle_Internal* Handle = VehicleInput.Proxy->GetPhysicsThreadAPI();
		if (Handle->ObjectState() != Chaos::EObjectStateType::Dynamic)
		{
			return;
		}

		bool bWake = false;
		OutputVehiclesBatch[Idx] = VehicleInput.Simulate(World, DeltaTime, SimTime, bWake);

	};

	bool ForceSingleThread = !GVehicleDebugParams.EnableMultithreading;
	PhysicsParallelFor(OutputVehiclesBatch.Num(), LambdaParallelUpdate, ForceSingleThread);

	// Delayed application of forces - This is separate from Simulate because forces cannot be executed multi-threaded
	for (const TUniquePtr<FChaosVehicleAsyncInput>& VehicleInput : InputVehiclesBatch)
	{
		if (VehicleInput.IsValid() && VehicleInput->Proxy)
		{
			if (Chaos::FRigidBodyHandle_Internal* Handle = VehicleInput->Proxy->GetPhysicsThreadAPI())
			{
				VehicleInput->ApplyDeferredForces(Handle);
			}
		}
	}
}

/**
 * Contact modification currently unused
 */
void FChaosHoveringVehicleManagerAsyncCallback::OnContactModification_Internal(Chaos::FCollisionContactModifier& Modifications)
{

}

FHoveringVehiclesBatch::~FHoveringVehiclesBatch() {
		
	while (Vehicles.Num() > 0)
	{
		FHoveringVehiclesBatch::RemoveVehicle(Vehicles.Last());
	}
}

void FHoveringVehiclesBatch::RegisterCallbacks(const FPhysScene_Chaos& scene) {
	check(AsyncCallback == nullptr)
	AsyncCallback = scene.GetSolver()->CreateAndRegisterSimCallbackObject_External<FChaosHoveringVehicleManagerAsyncCallback>();
}

void FHoveringVehiclesBatch::UnregisterCallbacks(const FPhysScene_Chaos& scene) {
		
	if (AsyncCallback)
	{
		scene.GetSolver()->UnregisterAndFreeSimCallbackObject_External(AsyncCallback);
		AsyncCallback = nullptr;
	}
}

void FHoveringVehiclesBatch::AddVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> vehicle) {
	if(auto veh = Cast<UChaosHoveringVehicleMovementComponent>(vehicle.Get())) {
		AddVehicle_Internal(veh);
	}
}

bool FHoveringVehiclesBatch::RemoveVehicle(TWeakObjectPtr<UChaosVehicleMovementComponent> vehicle) {
	check(vehicle != NULL);
	check(vehicle->HasValidPhysicsState());
	if(auto veh = Cast<UChaosHoveringVehicleMovementComponent>(vehicle.Get())) {
		return RemoveVehicle_Internal(veh);
	}
	return false;
}

int32 FHoveringVehiclesBatch::PostUpdate() {
	int32 SleepingCount = 0;
	for (int32 i = 0; i < Vehicles.Num(); ++i)
	{
		if (Vehicles[i]->VehicleState.bSleeping)
		{
			SleepingCount++;
		}
	}
	return SleepingCount;
}

void FHoveringVehiclesBatch::Update(float DeltaTime) {
	FChaosVehicleManagerAsyncInput* AsyncInput = AsyncCallback->GetProducerInputData_External();
	for (auto Vehicle : Vehicles)
	{
		Vehicle->Update(DeltaTime);
		Vehicle->FinalizeSimCallbackData(*AsyncInput);
	}
}

void FHoveringVehiclesBatch::PreTick(float DeltaTime) {
	for (int32 i = 0; i < Vehicles.Num(); ++i)
	{
		Vehicles[i]->PreTickGT(DeltaTime);
	}
}

void FHoveringVehiclesBatch::ParallelUpdate(int32& Timestamp, FPhysScene_Chaos& scene, float DeltaSeconds) {
	FChaosVehicleManagerAsyncInput* AsyncInput = AsyncCallback->GetProducerInputData_External();

	AsyncInput->Reset();	//only want latest frame's data

	{
		// We pass pointers from TArray so this reserve is critical. Otherwise realloc happens
		AsyncInput->VehicleInputs.Reserve(Vehicles.Num());
		AsyncInput->Timestamp = Timestamp;
		AsyncInput->World = scene.GetOwningWorld();
	}

	// Grab all outputs for processing, even future ones for interpolation.
	{
		Chaos::TSimCallbackOutputHandle<FChaosVehicleManagerAsyncOutput> AsyncOutputLatest;
		while ((AsyncOutputLatest = AsyncCallback->PopFutureOutputData_External()))
		{
			PendingOutputs.Emplace(MoveTemp(AsyncOutputLatest));
		}
	}

	// Since we are in pre-physics, delta seconds is not accounted for in external time yet
	const float ResultsTime = AsyncCallback->GetSolver()->GetPhysicsResultsTime_External() + DeltaSeconds;

	// Find index of first non-consumable output (first one after current time)
	int32 LastOutputIdx = 0;
	for (; LastOutputIdx < PendingOutputs.Num(); ++LastOutputIdx)
	{
		if (PendingOutputs[LastOutputIdx]->InternalTime > ResultsTime)
		{
			break;
		}
	}

	// Process events on all outputs which occurred before current time
	// 
	//for (int32 OutputIdx = 0; OutputIdx < LastOutputIdx; ++OutputIdx)
	//{
	//	for (TWeakObjectPtr<UChaosVehicleMovementComponent> Vehicle : Vehicles)
	//	{
	//		Vehicle->GameThread_ProcessIntermediateAsyncOutput(*PendingOutputs[OutputIdx]);
	//	}
	//}

	// Cache the last consumed output for interpolation
	if (LastOutputIdx > 0)
	{
		LatestOutput = MoveTemp(PendingOutputs[LastOutputIdx - 1]);
	}

	// Remove all consumed outputs
	{
		TArray<Chaos::TSimCallbackOutputHandle<FChaosVehicleManagerAsyncOutput>> NewPendingOutputs;
		for (int32 OutputIdx = LastOutputIdx; OutputIdx < PendingOutputs.Num(); ++OutputIdx)
		{
			NewPendingOutputs.Emplace(MoveTemp(PendingOutputs[OutputIdx]));
		}
		PendingOutputs = MoveTemp(NewPendingOutputs);
	}

	// It's possible we will end up multiple frames ahead of output, take the latest ready output.
	Chaos::TSimCallbackOutputHandle<FChaosVehicleManagerAsyncOutput> AsyncOutput;
	Chaos::TSimCallbackOutputHandle<FChaosVehicleManagerAsyncOutput> AsyncOutputLatest;
	while ((AsyncOutputLatest = AsyncCallback->PopOutputData_External()))
	{
		AsyncOutput = MoveTemp(AsyncOutputLatest);

		// Note: not used - left as a reminder
		//for (TWeakObjectPtr<UChaosVehicleMovementComponent> Vehicle : Vehicles)
		//{
		//	//Vehicle->GameThread_ProcessIntermediateAsyncOutput(*AsyncOutput);
		//}
	}

	if (UWorld* World = scene.GetOwningWorld())
	{
		int32 NumVehiclesInActiveBatch = 0;
		for (auto Vehicle : Vehicles)
		{
			auto NextOutput = PendingOutputs.Num() > 0 ? PendingOutputs[0].Get() : nullptr;
			float Alpha = 0.f;
			if (NextOutput && LatestOutput)
			{
				const float Denom = NextOutput->InternalTime - LatestOutput->InternalTime;
				if (Denom > SMALL_NUMBER)
				{
					Alpha = (ResultsTime - LatestOutput->InternalTime) / Denom;
				}
			}

			AsyncInput->VehicleInputs.Add(Vehicle->SetCurrentAsyncInputOutput(AsyncInput->VehicleInputs.Num(), LatestOutput.Get(), NextOutput, Alpha, Timestamp));
		}
	}

	++Timestamp;

	const auto& AwakeVehiclesBatch = Vehicles; // TODO: process awake only

	auto LambdaParallelUpdate = [DeltaSeconds, &AwakeVehiclesBatch](int32 Idx)
	{
		TWeakObjectPtr<UChaosVehicleMovementComponent> Vehicle = AwakeVehiclesBatch[Idx];
		Vehicle->ParallelUpdate(DeltaSeconds); // gets output state from PT
	};

	bool ForceSingleThread = !GVehicleDebugParams.EnableMultithreading;
	ParallelFor(AwakeVehiclesBatch.Num(), LambdaParallelUpdate, ForceSingleThread);
}

void FHoveringVehiclesBatch::AddVehicle_Internal(TWeakObjectPtr<UChaosHoveringVehicleMovementComponent> vehicle) {
	Vehicles.Add(vehicle);
}

bool FHoveringVehiclesBatch::RemoveVehicle_Internal(TWeakObjectPtr<UChaosHoveringVehicleMovementComponent> vehicle) {
	vehicle->PVehicleOutput.Reset();
	return Vehicles.Remove(vehicle) > 0;
}

