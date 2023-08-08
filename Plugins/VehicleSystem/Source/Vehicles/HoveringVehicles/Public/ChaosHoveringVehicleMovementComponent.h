// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "UObject/ObjectMacros.h"
#include "ChaosVehicleMovementComponent.h"
#include "ChaosHoveringVehicleManagerAsyncCallback.h"
#include "EngineSystem.h"
#include "SteeringSystem.h"
#include "SuspensionSystem.h"
#include "Curves/CurveFloat.h"
#include "VehicleUtility.h"
#include "ChaosVehicleMover.h"
#include "ElevatorSystem.h"
#include "WheelSystem.h"
#include "HoverSystem.h"
#include "PhysicsProxy/SingleParticlePhysicsProxyFwd.h"
#include "ChaosHoveringVehicleMovementComponent.generated.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif
struct FChaosVehicleAsyncInput;
struct FHoveringVehicleAsyncInput;
struct FChaosVehicleManagerAsyncInput;
struct FChaosVehicleManagerAsyncOutput;
struct FHoveringVehicleAsyncOutput;
struct FHoveringVehicleOutput;

extern FLogCategoryLogVehicle LogVehicle;

namespace Chaos {
	class FAxleSim;
	class FStabilizeControlSim;
	class FTargetRotationControlSim;
	class FTorqueControlSim;
	class FSimpleThrustSim;
	class FAerofoil;
	class FSimpleAerodynamicsSim;
	class FSimpleSteeringSim;
	class FSimpleEngineSim;
	class FSimpleSuspensionSim;
	class FSimpleWheelSim;
	class FSimpleDifferentialSim;
	class FSimpleTransmissionSim;

	class FSimpleHoveringVehicle : public IVehicleInterface
	{
	public:
		FSimpleHoveringVehicle()
		: bSuspensionEnabled(true)
		, bMechanicalSimEnabled(true)
		, bWheelFrictionEnabled(true)
		, bLegacyWheelFrictionPosition(false)
		{

		}
		virtual const bool IsGrounded(int MoverIndex = -1) override;

		virtual ~FSimpleHoveringVehicle()
		{

		}
	virtual TArray<FAerofoil>& GetAerofoils() override {return Aerofoils;}
		/*void Simulate(float DeltaTime) override
		{
		}*/

		bool IsValid()
		{
			return (Transmission.Num() == 1) && (Engine.Num() == 1) && (Aerodynamics.Num() == 1);
		}

		FSimpleEngineSim& GetEngine()
		{
			check(Engine.Num() == 1);
			return Engine[0];
		}

		bool HasEngine() const
		{
			return (Engine.Num() > 0);
		}

		virtual bool HasTransmission() const override
		{
			return (Transmission.Num() > 0);
		}

		virtual bool HasTorqueControlSetup() override
		{
			return (TorqueControlSim.Num() > 0);
		}

		bool HasTargetRotationControlSetup()
		{
			return (TargetRotationControlSim.Num() > 0);
		}

		bool HasStabilizeControlSetup()
		{
			return (StabilizeControlSim.Num() > 0);
		}


		virtual FSimpleTransmissionSim& GetTransmission() override
		{
			check(Transmission.Num() == 1);
			return Transmission[0];
		}

		FSimpleDifferentialSim& GetDifferential()
		{
			check(Differential.Num() == 1);
			return Differential[0];
		}
		FSimpleElevatorSim& GetElevator(int WheelIdx)
		{
			check(WheelIdx < Elevators.Num());
			return Elevators[WheelIdx];
		}
		FSimpleSuspensionSim& GetSuspension(int WheelIdx)
		{
			check(WheelIdx < Suspension.Num());
			return Suspension[WheelIdx];
		}

		FSimpleSteeringSim& GetSteering()
		{
			check(Steering.Num() == 1);
			return Steering[0];
		}

		virtual FSimpleAerodynamicsSim& GetAerodynamics() override
		{
			check(Aerodynamics.Num() == 1);
			return Aerodynamics[0];
		}
	
		FAerofoil& GetAerofoil(int AerofoilIdx)
		{
			check(AerofoilIdx < Aerofoils.Num());
			return Aerofoils[AerofoilIdx];
		}

		FSimpleThrustSim& GetThruster(int ThrusterIdx)
		{
			check(ThrusterIdx < Thrusters.Num());
			return Thrusters[ThrusterIdx];
		}

		virtual FTorqueControlSim& GetTorqueControl() override
		{
			check(TorqueControlSim.Num() == 1);
			return TorqueControlSim[0];
		}

		virtual FTargetRotationControlSim& GetTargetRotationControl() override
		{
			check(TargetRotationControlSim.Num() == 1);
			return TargetRotationControlSim[0];
		}

		FStabilizeControlSim& GetStabilizeControl()
		{
			check(StabilizeControlSim.Num() == 1);
			return StabilizeControlSim[0];
		}

		const TArray<FAxleSim>& GetAxles() const
		{
			return Axles;
		}

		virtual TArray<FSimpleThrustSim>& GetThrusters() override {
			return Thrusters;
		}


		TArray<FSimpleEngineSim> Engine;
		TArray<FSimpleHoverSim> Hovering;
		TArray<FSimpleTransmissionSim> Transmission;
		TArray<FSimpleDifferentialSim> Differential;
		TArray<FSimpleElevatorSim> Elevators;
		TArray<FSimpleSuspensionSim> Suspension;
		TArray<FSimpleSteeringSim> Steering;
		TArray<FSimpleAerodynamicsSim> Aerodynamics;
		TArray<FAerofoil> Aerofoils;
		TArray<FSimpleThrustSim> Thrusters;
		TArray<FAxleSim> Axles;

		TArray<FTorqueControlSim> TorqueControlSim;
		TArray<FTargetRotationControlSim> TargetRotationControlSim;
		TArray<FStabilizeControlSim> StabilizeControlSim;
		//TArray<FSimpleFuelSim> Fuel;
		//TArray<FSimpleBallastSim> Ballast;
		// turbo
		// .. 

		bool bSuspensionEnabled;
		bool bMechanicalSimEnabled;
		bool bWheelFrictionEnabled;
		bool bLegacyWheelFrictionPosition;
	};
}

USTRUCT()
struct HOVERINGVEHICLES_API FHoveringControlConfig
{
	GENERATED_USTRUCT_BODY()

	FHoveringControlConfig()
	{
		InitDefaults();
	}

	//Movement
	UPROPERTY(EditAnywhere, Category = Setup)
	float MaxSpeed;
	UPROPERTY(EditAnywhere, Category = Setup)
	float ForwardTorque; 
	UPROPERTY(EditAnywhere, Category = Setup)
	float ReverseTorque;
	UPROPERTY(EditAnywhere, Category = Setup)
	float Deceleration;

	UPROPERTY(EditAnywhere, Category = Setup)
	float MaxEquilibriumOffset;
	UPROPERTY(EditAnywhere, Category = Setup)
	float DefaultSuspensionLength;

	//Control
	UPROPERTY(EditAnywhere, Category = Setup)
	float SideMomentumForce;

	//Steering
	UPROPERTY(EditAnywhere, Category = Setup)
	float Steering;

	const Chaos::FSimpleHoverConfig& GetPhysicsHoverConfig()
	{
		FillHoverSetup();
		return PHoverConfig;
	}

	void InitDefaults()
	{
		MaxSpeed = 4000.0f;
		ForwardTorque = 1.0f;
		ReverseTorque = 0.25f;
		Deceleration = 1.f;
		MaxEquilibriumOffset = 200.0f;
		SideMomentumForce = 2000.f;
		Steering = 20;
	}
	
private:

	void FillHoverSetup()
	{
		PHoverConfig.MaxSpeed = MaxSpeed;
		PHoverConfig.ForwardTorque = ForwardTorque;
		PHoverConfig.ReverseTorque = ReverseTorque;
		PHoverConfig.Deceleration = Deceleration;
		PHoverConfig.SideMomentumForce = SideMomentumForce;
		PHoverConfig.Steering = Steering;
	}

	Chaos::FSimpleHoverConfig PHoverConfig;

};

extern FVehicleDebugParams GVehicleDebugParams;
struct HOVERINGVEHICLES_API FHoveringVehicleDebugParams
{
	bool ShowWheelCollisionNormal = false;
	bool ShowSuspensionRaycasts = false;
	bool ShowSuspensionLimits = false;
	bool ShowWheelForces = false;
	bool ShowSuspensionForces = false;
	bool ShowBatchQueryExtents = false;
	bool ShowRaycastComponent = false;
	bool ShowRaycastMaterial = false;
	int TraceTypeOverride = 0;

	bool DisableSuspensionForces = false;
	bool DisableFrictionForces = false;
	bool DisableRollbarForces = false;
	bool DisableConstraintSuspension = false;

	float ThrottleOverride = 0.f;
	float SteeringOverride = 0.f;

	bool ResetPerformanceMeasurements = false;

	float OverlapTestExpansionXY = 100.f;
	float OverlapTestExpansionZ = 50.f;
};

/**
 * There is too much information for one screen full of debug data, so sub-pages of information are available 
 * Advance through pages using p.Vehicles.NextDebugPage | p.Vehicles.PrevDebugPage which can be hooked
 * up to the keyboard or a controller in blueprint using execCommand
 */


/**
 * Structure containing information about the status of a single wheel of the vehicle.
 */
USTRUCT(BlueprintType)
struct HOVERINGVEHICLES_API FElevatorStatus
{
	GENERATED_BODY()

	/** This wheel is in contact with the ground */
	UPROPERTY()
	bool bInContact;

	/** Wheel contact point */
	UPROPERTY()
	FVector ContactPoint;

	/** Material that wheel is in contact with */
	UPROPERTY()
	TWeakObjectPtr<class UPhysicalMaterial> PhysMaterial;

	/** Normalized suspension length at this wheel */
	UPROPERTY()
	float NormalizedSuspensionLength;

	/** Spring Force that is occurring at wheel suspension */
	UPROPERTY()
	float SpringForce;

	FElevatorStatus()
	{
		Init();
	}

	explicit FElevatorStatus(EForceInit InInit)
	{
		Init();
	}

	explicit FElevatorStatus(ENoInit NoInit)
	{
		bIsValid = false;
	}

	void Init()
	{
		bInContact = false;
		NormalizedSuspensionLength = 1.f;
		SpringForce = 0.f;
		ContactPoint = FVector::ZeroVector;
		bIsValid = false;
	}

	FString ToString() const;

	bool bIsValid;
};

USTRUCT()
struct HOVERINGVEHICLES_API FChaosElevatorSetup
{
	GENERATED_USTRUCT_BODY()

	// The wheel class to use
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	TSubclassOf<UChaosVehicleElevator> WheelClass;

	// Bone name on mesh to create wheel at
	//UPROPERTY(EditAnywhere, Category = WheelSetup)
	//FName SteeringBoneName;

	// Bone name on mesh to create wheel at
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	FName BoneName;

	// Additional offset to give the wheels for this axle.
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	FVector AdditionalOffset;

	FChaosElevatorSetup();
};

/** Commonly used Wheel state - evaluated once used wherever required for that frame */
struct HOVERINGVEHICLES_API FElevatorState
{
	void Init(int NumWheels)
	{
		WheelLocalLocation.Init(FVector::ZeroVector, NumWheels);
		WheelWorldLocation.Init(FVector::ZeroVector, NumWheels);
		WorldWheelVelocity.Init(FVector::ZeroVector, NumWheels);
		LocalWheelVelocity.Init(FVector::ZeroVector, NumWheels);
		Trace.SetNum(NumWheels);
		TraceResult.SetNum(NumWheels);
	}

	/** Commonly used Wheel state - evaluated once used wherever required for that frame */
	void CaptureState(int WheelIdx, const FVector& WheelOffset, const FBodyInstance* TargetInstance);
	void CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* Handle);
	void CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* VehicleHandle, const FVector& ContactPoint, const Chaos::FRigidBodyHandle_Internal* SurfaceHandle);
	static FVector GetVelocityAtPoint(const Chaos::FRigidBodyHandle_Internal* Rigid, const FVector& InPoint);

	TArray<FVector> WheelLocalLocation;	/** Current Location Of Wheels In Local Coordinates */
	TArray<FVector> WheelWorldLocation;	/** Current Location Of Wheels In World Coordinates */
	TArray<FVector> WorldWheelVelocity; /** Current velocity at wheel location In World Coordinates - combined linear and angular */
	TArray<FVector> LocalWheelVelocity; /** Local velocity of Wheel */
	TArray<Chaos::FSuspensionTrace> Trace;
	TArray<FHitResult> TraceResult;
};

//////////////////////////////////////////////////////////////////////////

class HOVERINGVEHICLES_API UChaosHoveringVehicleSimulation : public UChaosVehicleSimulation
{
public:

	UChaosHoveringVehicleSimulation()
		: bOverlapHit(false)
	{
		QueryBox.Init();
	}

	virtual ~UChaosHoveringVehicleSimulation()
	{
		PVehicle = nullptr;
	}

	virtual void Init(TUniquePtr<Chaos::FSimpleHoveringVehicle>& PVehicleIn)
	{
		PVehicle = MoveTemp(PVehicleIn);

		WheelState.Init(PVehicle->Elevators.Num());
	}

	virtual void UpdateConstraintHandles(TArray<FPhysicsConstraintHandle>& ConstraintHandlesIn) override;
	/** Reinitialize a wheel at runtime */
	void InitializeElevator(int WheelIndex, const Chaos::FSimpleElevatorConfig* InWheelSetup);

	/** Reinitialize the physics suspension at runtime */
	void InitializeSuspension(int WheelIndex, const Chaos::FSimpleSuspensionConfig* InSuspensionSetup);

	virtual void TickVehicle(UWorld* WorldIn, float DeltaTime, const FHoveringVehicleAsyncInput& InputData, FHoveringVehicleAsyncOutput& OutputData, Chaos::FRigidBodyHandle_Internal* Handle);

	/** Advance the vehicle simulation */
	virtual void UpdateSimulation(float DeltaTime, const FHoveringVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle);

	virtual void FillOutputState(FHoveringVehicleAsyncOutput& Output);

	/** Are enough vehicle systems specified such that physics vehicle simulation is possible */
	virtual bool CanSimulate() const override;

	/** Pass control Input to the vehicle systems */
	virtual void ApplyInput(const FControlInputs& ControlInputs, float DeltaTime) override;

	/** Perform suspension ray/shape traces */
	virtual void PerformSuspensionTraces(const TArray<Chaos::FSuspensionTrace>& SuspensionTrace, FCollisionQueryParams& TraceParams, FCollisionResponseContainer& CollisionResponse, TArray<FWheelTraceParams>& WheelTraceParams);


	/** Update the engine/transmission simulation */
	virtual void ProcessMechanicalSimulation(float DeltaTime);

	/** Process steering mechanism */
	virtual void ProcessSteering(const FControlInputs& ControlInputs);

	/** calculate and apply lateral and longitudinal friction forces from wheels */
	virtual void ApplyWheelFrictionForces(float DeltaTime);

	/** calculate and apply chassis suspension forces */
	virtual void ApplySuspensionForces(float DeltaTime);

	bool ContainsTraces(const FBox& Box, const TArray<struct Chaos::FSuspensionTrace>& SuspensionTrace);


	/** Draw 3D debug lines and things along side the 3D model */
	virtual void DrawDebug3D() override;

	FElevatorState WheelState;	/** Cached state that holds wheel data for this frame */

	TArray<FPhysicsConstraintHandle> ConstraintHandles;

	// cache trace overlap query
	TArray<FOverlapResult> OverlapResults;
	bool bOverlapHit;
	FBox QueryBox;
	TUniquePtr<Chaos::FSimpleHoveringVehicle> PVehicle;
	virtual TObjectPtr<Chaos::IVehicleInterface> GetPVehicle() override {return PVehicle.Get();}
};

//////////////////////////////////////////////////////////////////////////

UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent), hidecategories = (PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class HOVERINGVEHICLES_API UChaosHoveringVehicleMovementComponent : public UChaosVehicleMovementComponent
{
	GENERATED_UCLASS_BODY()

	UPROPERTY(EditAnywhere, Category = WheelSetup)
	bool bSuspensionEnabled;

	UPROPERTY(EditAnywhere, Category = WheelSetup)
	bool bWheelFrictionEnabled;

	UPROPERTY(EditAnywhere, Category = WheelSetup)
	bool bLegacyWheelFrictionPosition;

	/** Wheels to create */
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	TArray<FChaosElevatorSetup> WheelSetups;

	UPROPERTY(EditAnywhere, Category = Custom)
	struct FCollisionResponseContainer WheelTraceCollisionResponses;

	UPROPERTY(EditAnywhere, Category = MechanicalSetup)
	bool bMechanicalSimEnabled;

	UPROPERTY(EditAnywhere, Category = MechanicalSetup, meta = (EditCondition = "bMechanicalSimEnabled"))
	FHoveringControlConfig HoverSetup;
	/** Engine */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup, meta = (EditCondition = "bMechanicalSimEnabled"))
	FVehicleEngineConfig EngineSetup;

	/** Differential */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup, meta = (EditCondition = "bMechanicalSimEnabled"))
	FVehicleDifferentialConfig DifferentialSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = MechanicalSetup, meta = (EditCondition = "bMechanicalSimEnabled"))
	FVehicleTransmissionConfig TransmissionSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = SteeringSetup)
	FVehicleSteeringConfig SteeringSetup;

	// Our instanced wheels
	UPROPERTY(transient, duplicatetransient, BlueprintReadOnly, Category = Vehicle)
	TArray<TObjectPtr<class UChaosVehicleElevator>> Wheels;

	/** Get current engine's rotation speed */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	float GetEngineRotationSpeed() const;

	/** Get current engine's max rotation speed */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	float GetEngineMaxRotationSpeed() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	int GetNumWheels() const 
	{
		return WheelStatus.Num();
	}
	
	virtual bool HasValidPhysicsState() const override;
	TUniquePtr<FHoveringVehicleOutput> PVehicleOutput;	/* physics simulation data output from the async physics thread */


	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelStatus(const struct FElevatorStatus& Status, bool& bInContact, FVector& ContactPoint, UPhysicalMaterial*& PhysMaterial
			, float& NormalizedSuspensionLength, float& SpringForce, float& SlipAngle, bool& bIsSlipping, float& SlipMagnitude, bool& bIsSkidding, float& SkidMagnitude, FVector& SkidNormal);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FElevatorStatus MakeWheelStatus(bool bInContact, FVector& ContactPoint, UPhysicalMaterial* PhysMaterial
			, float NormalizedSuspensionLength, float SpringForce, float SlipAngle, bool bIsSlipping, float SlipMagnitude, bool bIsSkidding, float SkidMagnitude, FVector& SkidNormal);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheeledSnapshot(const struct FHoveringSnapshotData& Snapshot, FTransform& Transform, FVector& LinearVelocity
			, FVector& AngularVelocity, int& SelectedGear, float& EngineRPM, TArray<FElevatorSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FHoveringSnapshotData MakeWheeledSnapshot(FTransform Transform, FVector LinearVelocity, FVector AngularVelocity
			, int SelectedGear, float EngineRPM, const TArray<FElevatorSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelSnapshot(const struct FElevatorSnapshot& Snapshot, float& SuspensionOffset
			, float& WheelRotationAngle, float& SteeringAngle, float& WheelRadius, float& WheelAngularVelocity);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FElevatorSnapshot MakeWheelSnapshot(float SuspensionOffset, float WheelRotationAngle
			, float SteeringAngle, float WheelRadius, float WheelAngularVelocity);

	/** Get a wheels current simulation state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	const FElevatorStatus& GetWheelState(int WheelIndex) const
	{
		return WheelStatus[WheelIndex];
	}

	virtual float GetSuspensionOffset(int WheelIndex) override;
	
	//----ASYNC----
	TUniquePtr<FChaosVehicleAsyncInput> SetCurrentAsyncInputOutput(int32 InputIdx, FChaosVehicleManagerAsyncOutput* CurOutput, FChaosVehicleManagerAsyncOutput* NextOutput, float Alpha, int32 VehicleManagerTimestamp);

	void SetCurrentAsyncInputOutputInternal(FChaosVehicleAsyncInput* CurInput, int32 InputIdx, FChaosVehicleManagerAsyncOutput* CurOutput, int32 VehicleManagerTimestamp);
	void SetCurrentAsyncInputOutputInternal(FChaosVehicleAsyncInput* CurInput, int32 InputIdx, FChaosVehicleManagerAsyncOutput* CurOutput, FChaosVehicleManagerAsyncOutput* NextOutput, float Alpha, int32 VehicleManagerTimestamp);

	
	UPhysicalMaterial* GetPhysMaterial(int WheelIndex);

	/** Set all channels to the specified response - for wheel raycasts */
	void SetWheelTraceAllChannels(ECollisionResponse NewResponse)
	{
		WheelTraceCollisionResponses.SetAllChannels(NewResponse);
	}

	/** Set the response of this body to the supplied settings - for wheel raycasts */
	void SetWheelTraceResponseToChannel(ECollisionChannel Channel, ECollisionResponse NewResponse)
	{
		WheelTraceCollisionResponses.SetResponse(Channel, NewResponse);
	}

	/** Get Collision ResponseToChannels container for this component **/
//	FORCEINLINE_DEBUGGABLE const FCollisionResponseContainer& GetTraceResponseToChannels() const { return WheelTraceCollisionResponses.GetResponseContainer(); }

	//////////////////////////////////////////////////////////////////////////
	// Public

	virtual void Serialize(FArchive& Ar) override;
	virtual void PostLoad() override;

	virtual void UpdateGroundedState(float DeltaTime) override;
	// Get output data from Physics Thread
	virtual void ParallelUpdate(float DeltaSeconds);

	EChaosAsyncVehicleDataType CurAsyncType;
	FChaosVehicleAsyncInput* CurAsyncInput;
	struct FHoveringVehicleAsyncOutput* CurAsyncOutput;
	struct FHoveringVehicleAsyncOutput* NextAsyncOutput;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	/** Are the configuration references configured sufficiently that the vehicle can be created */
	virtual bool CanCreateVehicle() const override;

	/** Used to create any physics engine information for this component */
	virtual void OnCreatePhysicsState() override;

	/** Used to shut down and pysics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;
	virtual void SetTargetGear(int32 GearNum, bool bImmediate) override;
	virtual int32 GetCurrentGear() const override;
	virtual void ClearRawInput() override;

	/** Enable or completely bypass the ProcessMechanicalSimulation call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void EnableMechanicalSim(bool InState)
	{
		bMechanicalSimEnabled = InState;
	}

	/** Enable or completely bypass the ApplySuspensionForces call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void EnableSuspension(bool InState)
	{
		bSuspensionEnabled = InState;
	}

	/** Enable or completely bypass the ApplyWheelFrictionForces call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void EnableWheelFriction(bool InState)
	{
		bWheelFrictionEnabled = InState;
	}

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelClass(int WheelIndex, TSubclassOf<UChaosVehicleElevator> InWheelClass);

	/** Grab a snapshot of the vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	virtual FHoveringSnapshotData GetSnapshot() const;

	/** Set snapshot of vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	virtual void SetSnapshot(const FBaseSnapshotData& SnapshotIn);


	//////////////////////////////////////////////////////////////////////////
	// change handling via blueprint at runtime
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetMaxEngineTorque(float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetDragCoefficient(float DragCoeff);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetDownforceCoefficient(float DownforceCoeff);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetDifferentialFrontRearSplit(float FrontRearSlpit);

	UFUNCTION(BlueprintCallable,BlueprintPure, Category = "Game|Components|ChaosWheeledVehicleMovement")
	float GetFloatProperty(FString id);
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetFloatProperty(FString id,float value);
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelRadius(int WheelIndex, float Radius);

	/** */
	virtual TUniquePtr<Chaos::FSimpleHoveringVehicle> CreatePhysicsVehicle() ;

	/** Allocate and setup the Chaos vehicle */
	virtual void SetupVehicle(TUniquePtr<Chaos::FSimpleHoveringVehicle>& PVehicle);

	virtual void ResetVehicleState() override;

protected:

	//////////////////////////////////////////////////////////////////////////
	// Setup

	/** Re-Compute any runtime constants values that rely on setup data */
	virtual void ComputeConstants() override;

	/** Skeletal mesh needs some special handling in the vehicle case */
	virtual void FixupSkeletalMesh();

	/** Create and setup the Chaos vehicle */
	virtual void CreateVehicle();

	/** Instantiate and setup our wheel objects */
	virtual void CreateWheels();

	/** Release our wheel objects */
	virtual void DestroyWheels();

	/** Set up the chassis and wheel shapes */
	virtual void SetupVehicleShapes();
	/** Setup calculated suspension parameters */
	void SetupSuspension(TUniquePtr<Chaos::FSimpleHoveringVehicle>& PVehicle);

	/** Maps UChaosVehicleElevator Axle to a wheel index */
	void RecalculateAxles();

	/** Get the local position of the wheel at rest */
	virtual FVector GetWheelRestingPosition(const FChaosElevatorSetup& WheelSetup);

	//////////////////////////////////////////////////////////////////////////
	// Update
	void FillWheelOutputState();

	/* Fill Async input state */


	//////////////////////////////////////////////////////////////////////////
	// Debug

	/** Draw 2D debug text graphs on UI for the wheels, suspension and other systems */
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos);

	/** Get distances between wheels - primarily a debug display helper */
	const FVector2D& GetWheelLayoutDimensions() const
	{
		return WheelTrackDimensions;
	}
	public:
	virtual bool IsOnAnySurface(const TArray<UPhysicalMaterial*> surfaces) override;
	void FinalizeSimCallbackData(FChaosVehicleManagerAsyncInput& Input);
		virtual void Update(float DeltaTime) override;
		virtual TObjectPtr<UChaosVehicleSimulation> GetVehicleSimulationPT() override {return VehicleSimulationPT;}
		TObjectPtr<UChaosHoveringVehicleSimulation> VehicleSimulationPT;	/* simulation code running on the physics thread async callback */
	private:


	/** Get distances between wheels - primarily a debug display helper */
	FVector2D CalculateWheelLayoutDimensions();

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	float CalcDialAngle(float CurrentValue, float MaxValue);
	void DrawDial(UCanvas* Canvas, FVector2D Pos, float Radius, float CurrentValue, float MaxValue);
#endif

	struct FCachedState
	{
		FCachedState() : WheelOffset(0.f), bIsValid(false)
		{ }

		float WheelOffset;
		bool bIsValid;
	};

	FVector2D WheelTrackDimensions;	// Wheelbase (X) and track (Y) dimensions
	TMap<UChaosVehicleElevator*, TArray<int>> AxleToWheelMap;
	TArray<FPhysicsConstraintHandle> ConstraintHandles;
	TArray<FElevatorStatus> WheelStatus; /** Wheel output status */
	TArray<FCachedState> CachedState;
	Chaos::FPerformanceMeasure PerformanceMeasure;
};

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
