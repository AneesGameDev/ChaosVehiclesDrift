// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "UObject/ObjectMacros.h"
#include "ChaosVehicleMovementComponent.h"
#include "EngineSystem.h"
#include "SteeringSystem.h"
#include "SuspensionSystem.h"
#include "Curves/CurveFloat.h"
#include "VehicleUtility.h"
#include "WheeledVehiclePawn.h"
#include "WheelSystem.h"
#include "ChaosWheeledVehicleManagerAsyncCallback.h"
#include "Chaos/PBDSuspensionConstraints.h"
#include "PhysicsProxy/SingleParticlePhysicsProxyFwd.h"
#include "ChaosWheeledVehicleMovementComponent.generated.h"


#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif



USTRUCT(BlueprintType)
struct WHEELEDVEHICLES_API FWheeledVehiclePreset
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	float DownforceCoefficient = 0.3;
	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	float FrontRearSplit = 0.2;
	UPROPERTY(EditAnyWhere , BlueprintReadWrite, Category = "DriftValue")
	TArray<float> WheelsMaxSteerAngle;
	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	TArray<float> WheelLoadRatio;
	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	TArray<float> WheelSideSlipModifier;
	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	TArray<float> WheelCorneringStiffness;
	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	TArray<float> WheelFrictionMultiplier;
	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	TArray<FRuntimeFloatCurve> WheelLateralSlipGraph;
	UPROPERTY(EditAnywhere , BlueprintReadWrite, Category = "DriftValue")
	TArray<bool> bWheelTractionControll;

	
};





struct FChaosVehicleAsyncInput;
struct FWheeledVehicleAsyncInput;
struct FChaosVehicleManagerAsyncInput;
struct FChaosVehicleManagerAsyncOutput;
struct FWheeledVehicleAsyncOutput;
struct FWheeledSnapshotData;
struct FWheelSnapshot;
struct FWheelTraceParams;
struct FWheeledVehicleOutput;


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

	class FSimpleWheeledVehicle : public IVehicleInterface
	{
	public:
		FSimpleWheeledVehicle()
		: bSuspensionEnabled(true)
		, bMechanicalSimEnabled(true)
		, bWheelFrictionEnabled(true)
		, NumDrivenWheels(0)
		, bLegacyWheelFrictionPosition(false)
		{

		}
		virtual const bool IsGrounded(int MoverIndex) override;

		virtual ~FSimpleWheeledVehicle()
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

		FSimpleWheelSim& GetWheel(int WheelIdx)
		{
			check(WheelIdx < Wheels.Num());
			return Wheels[WheelIdx];
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
		TArray<FSimpleTransmissionSim> Transmission;
		TArray<FSimpleDifferentialSim> Differential;
		TArray<FSimpleWheelSim> Wheels;
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
		uint32 NumDrivenWheels;
		bool bLegacyWheelFrictionPosition;
	};
}



extern FVehicleDebugParams GVehicleDebugParams;
struct WHEELEDVEHICLES_API FWheeledVehicleDebugParams
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
struct WHEELEDVEHICLES_API FWheelStatus
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

	/** Slip angle at the wheel - difference between wheel local direction and velocity at wheel */
	UPROPERTY()
	float SlipAngle;

	/** Is the wheel slipping */
	UPROPERTY()
	bool bIsSlipping;

	/** Magnitude of slippage of wheel, difference between wheel speed and ground speed */
	UPROPERTY()
	float SlipMagnitude;


	/** Is the wheel skidding */
	UPROPERTY()
	bool bIsSkidding;

	/** Magnitude of skid */
	UPROPERTY()
	float SkidMagnitude;

	/** Direction of skid, i.e. normalized direction */
	UPROPERTY()
	FVector SkidNormal;

	FWheelStatus()
	{
		Init();
	}

	explicit FWheelStatus(EForceInit InInit)
	{
		Init();
	}

	explicit FWheelStatus(ENoInit NoInit)
	{
		bIsValid = false;
	}

	void Init()
	{
		SlipAngle = 0.0f;
		bInContact = false;
		bIsSlipping = false;
		bIsSkidding = false;
		SlipMagnitude = 0.f;
		SkidMagnitude = 0.f;
		NormalizedSuspensionLength = 1.f;
		SpringForce = 0.f;
		SkidNormal = FVector::ZeroVector;
		ContactPoint = FVector::ZeroVector;
		bIsValid = false;
	}

	FString ToString() const;

	bool bIsValid;
};

USTRUCT()
struct WHEELEDVEHICLES_API FChaosWheelSetup
{
	GENERATED_USTRUCT_BODY()

	// The wheel class to use
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	TSubclassOf<UChaosVehicleWheel> WheelClass;

	// Bone name on mesh to create wheel at
	//UPROPERTY(EditAnywhere, Category = WheelSetup)
	//FName SteeringBoneName;

	// Bone name on mesh to create wheel at
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	FName BoneName;

	// Additional offset to give the wheels for this axle.
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	FVector AdditionalOffset;

	FChaosWheelSetup();
};

/** Commonly used Wheel state - evaluated once used wherever required for that frame */
struct WHEELEDVEHICLES_API FWheelState
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

class WHEELEDVEHICLES_API UChaosWheeledVehicleSimulation : public UChaosVehicleSimulation
{
public:

	bool isDrifting;
	bool isPressed = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite , Category = Presets)
	FWheeledVehiclePreset GripSetup;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite , Category = Presets)
	FWheeledVehiclePreset DriftSetup;

	UChaosWheeledVehicleSimulation()
		: bOverlapHit(false)
	{
		QueryBox.Init();
	}

	virtual ~UChaosWheeledVehicleSimulation()
	{
		PVehicle = nullptr;
	}

	virtual void Init(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicleIn)
	{
		PVehicle = MoveTemp(PVehicleIn);

		WheelState.Init(PVehicle->Wheels.Num());
	}

	virtual void UpdateConstraintHandles(TArray<FPhysicsConstraintHandle>& ConstraintHandlesIn) override;
	/** Reinitialize a wheel at runtime */
	void InitializeWheel(int WheelIndex, const Chaos::FSimpleWheelConfig* InWheelSetup);

	/** Reinitialize the physics suspension at runtime */
	void InitializeSuspension(int WheelIndex, const Chaos::FSimpleSuspensionConfig* InSuspensionSetup);

	virtual void TickVehicle(UWorld* WorldIn, float DeltaTime, const FWheeledVehicleAsyncInput& InputData, FWheeledVehicleAsyncOutput& OutputData, Chaos::FRigidBodyHandle_Internal* Handle);

	void UpdateDriftingState(const FControlInputs& control_inputs);
	/** Advance the vehicle simulation */
	virtual void UpdateSimulation(float DeltaTime, const FWheeledVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle);

	virtual void FillOutputState(FWheeledVehicleAsyncOutput& Output);

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

	bool IsWheelSpinning() const;
	bool ContainsTraces(const FBox& Box, const TArray<struct Chaos::FSuspensionTrace>& SuspensionTrace);


	/** Draw 3D debug lines and things along side the 3D model */
	virtual void DrawDebug3D() override;

	FWheelState WheelState;	/** Cached state that holds wheel data for this frame */

	TArray<FPhysicsConstraintHandle> ConstraintHandles;

	// cache trace overlap query
	TArray<FOverlapResult> OverlapResults;
	bool bOverlapHit;
	FBox QueryBox;
	TUniquePtr<Chaos::FSimpleWheeledVehicle> PVehicle;
	virtual TObjectPtr<Chaos::IVehicleInterface> GetPVehicle() override {return PVehicle.Get();}
};

//////////////////////////////////////////////////////////////////////////

UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent), hidecategories = (PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class WHEELEDVEHICLES_API UChaosWheeledVehicleMovementComponent : public UChaosVehicleMovementComponent
{
	GENERATED_UCLASS_BODY()
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite , Category = Presets)
	FWheeledVehiclePreset GripSetup;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite , Category = Presets)
	FWheeledVehiclePreset DriftSetup;

	
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	bool bSuspensionEnabled;

	UPROPERTY(EditAnywhere, Category = WheelSetup)
	bool bWheelFrictionEnabled;

	UPROPERTY(EditAnywhere, Category = WheelSetup)
	bool bLegacyWheelFrictionPosition;

	/** Wheels to create */
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	TArray<FChaosWheelSetup> WheelSetups;

	UPROPERTY(EditAnywhere, Category = Custom)
	struct FCollisionResponseContainer WheelTraceCollisionResponses;

	UPROPERTY(EditAnywhere, Category = MechanicalSetup)
	bool bMechanicalSimEnabled;

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
	TArray<TObjectPtr<class UChaosVehicleWheel>> Wheels;

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
	virtual bool IsOnAnySurface(const TArray<UPhysicalMaterial*> surfaces) override;
	virtual bool HasValidPhysicsState() const override;
	TUniquePtr<FWheeledVehicleOutput> PVehicleOutput;	/* physics simulation data output from the async physics thread */

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelStatus(const struct FWheelStatus& Status, bool& bInContact, FVector& ContactPoint, UPhysicalMaterial*& PhysMaterial
			, float& NormalizedSuspensionLength, float& SpringForce, float& SlipAngle, bool& bIsSlipping, float& SlipMagnitude, bool& bIsSkidding, float& SkidMagnitude, FVector& SkidNormal);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FWheelStatus MakeWheelStatus(bool bInContact, FVector& ContactPoint, UPhysicalMaterial* PhysMaterial
			, float NormalizedSuspensionLength, float SpringForce, float SlipAngle, bool bIsSlipping, float SlipMagnitude, bool bIsSkidding, float SkidMagnitude, FVector& SkidNormal);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheeledSnapshot(const FWheeledSnapshotData& Snapshot, FTransform& Transform, FVector& LinearVelocity
			, FVector& AngularVelocity, int& SelectedGear, float& EngineRPM, TArray<FWheelSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FWheeledSnapshotData MakeWheeledSnapshot(FTransform Transform, FVector LinearVelocity, FVector AngularVelocity
			, int SelectedGear, float EngineRPM, const TArray<FWheelSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelSnapshot(const struct FWheelSnapshot& Snapshot, float& SuspensionOffset
			, float& WheelRotationAngle, float& SteeringAngle, float& WheelRadius, float& WheelAngularVelocity);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FWheelSnapshot MakeWheelSnapshot(float SuspensionOffset, float WheelRotationAngle
			, float SteeringAngle, float WheelRadius, float WheelAngularVelocity);

	/** Get a wheels current simulation state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	const FWheelStatus& GetWheelState(int WheelIndex) const
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

	enum EChaosAsyncVehicleDataType CurAsyncType;
	FChaosVehicleAsyncInput* CurAsyncInput;
	struct FWheeledVehicleAsyncOutput* CurAsyncOutput;
	struct FWheeledVehicleAsyncOutput* NextAsyncOutput;
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
	/** display next debug page */
	static void NextDebugPage();

	/** display previous debug page */
	static void PrevDebugPage();

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
	void SetWheelClass(int WheelIndex, TSubclassOf<UChaosVehicleWheel> InWheelClass);

	/** Grab a snapshot of the vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	virtual FWheeledSnapshotData GetSnapshot() const;

	/** Set snapshot of vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	virtual void SetSnapshot(const FWheeledSnapshotData& SnapshotIn);


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

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetTractionControlEnabled(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetABSEnabled(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetAffectedByBrake(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetAffectedByHandbrake(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetAffectedBySteering(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetAffectedByEngine(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelRadius(int WheelIndex, float Radius);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelFrictionMultiplier(int WheelIndex, float Friction);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelSlipGraphMultiplier(int WheelIndex, float Multiplier);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelMaxBrakeTorque(int WheelIndex, float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelHandbrakeTorque(int WheelIndex, float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelMaxSteerAngle(int WheelIndex, float AngleDegrees);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetTorqueCombineMethod(ETorqueCombineMethod InCombineMethod, int32 WheelIndex);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetDriveTorque(float DriveTorque, int32 WheelIndex);
	
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetBrakeTorque(float BrakeTorque, int32 WheelIndex);


	
	/**Start Customize Drift Functions ---------------------------------------------------------*/

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelLoadRatio(int32 WheelIndex, float LoadRatio);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelSlideSlipModifier( int32 WheelIndex ,float SlipModifier);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelCorneringStiffness( int32 WheelIndex , float CorneringStiff);

	/*UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
		void SetWheelFrictionMultiplier(float Friction, int32 WheelIndex);*/

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosWheeledVehicleMovement")
	void SetWheelLateralSlipGraph(int32 WheelIndex, FRuntimeFloatCurve LateralSlipGraph);
	
	/**End Customize Drift Functions ---------------------------------------------------------*/



	
	/** */
	virtual TUniquePtr<Chaos::FSimpleWheeledVehicle> CreatePhysicsVehicle() ;

	/** Allocate and setup the Chaos vehicle */
	virtual void SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle);

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
	void SetupSuspension(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle);

	/** Maps UChaosVehicleWheel Axle to a wheel index */
	void RecalculateAxles();

	/** Get the local position of the wheel at rest */
	virtual FVector GetWheelRestingPosition(const FChaosWheelSetup& WheelSetup);

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
	void FinalizeSimCallbackData(FChaosVehicleManagerAsyncInput& Input);
		virtual void Update(float DeltaTime) override;
		virtual TObjectPtr<UChaosVehicleSimulation> GetVehicleSimulationPT() override {return VehicleSimulationPT;}
		TObjectPtr<UChaosWheeledVehicleSimulation> VehicleSimulationPT;	/* simulation code running on the physics thread async callback */
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

	uint32 NumDrivenWheels; /** The number of wheels that have engine enabled checked */
	FVector2D WheelTrackDimensions;	// Wheelbase (X) and track (Y) dimensions
	TMap<UChaosVehicleWheel*, TArray<int>> AxleToWheelMap;
	TArray<FPhysicsConstraintHandle> ConstraintHandles;
	TArray<FWheelStatus> WheelStatus; /** Wheel output status */
	TArray<FCachedState> CachedState;
	Chaos::FPerformanceMeasure PerformanceMeasure;
};

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
