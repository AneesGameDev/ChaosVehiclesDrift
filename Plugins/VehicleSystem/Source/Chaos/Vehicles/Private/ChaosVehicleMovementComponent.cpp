// Copyright Epic Games, Inc. All Rights Reserved.

#include "ChaosVehicleMovementComponent.h"
#include "EngineGlobals.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"
#include "CanvasItem.h"
#include "Engine/Canvas.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "DrawDebugHelpers.h"
#include "UObject/FrameworkObjectVersion.h"
#include "Net/UnrealNetwork.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Physics/PhysicsFiltering.h"
#include "Physics/PhysicsInterfaceUtils.h"
#include "GameFramework/PawnMovementComponent.h"
#include "Logging/MessageLog.h"
#include "DisplayDebugHelpers.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PBDJointConstraintData.h"
#include "Chaos/DebugDrawQueue.h"

#include "ChaosVehicleManager.h"
#include "SimpleVehicle.h"

#include "AI/Navigation/AvoidanceManager.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "GameFramework/HUD.h"

#include "Chaos/Particle/ParticleUtilities.h"
#include "Chaos/ParticleHandleFwd.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"


#define LOCTEXT_NAMESPACE "UVehicleMovementComponent"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif




FAutoConsoleCommand CVarCommandVehiclesNextDebugPage(
	TEXT("p.Vehicle.NextDebugPage"),
	TEXT("Display the next page of vehicle debug data."),
	FConsoleCommandDelegate::CreateStatic(UChaosVehicleMovementComponent::NextDebugPage));

FAutoConsoleCommand CVarCommandVehiclesPrevDebugPage(
	TEXT("p.Vehicle.PrevDebugPage"),
	TEXT("Display the previous page of vehicle debug data."),
	FConsoleCommandDelegate::CreateStatic(UChaosVehicleMovementComponent::PrevDebugPage));


FAutoConsoleVariableRef CVarChaosVehiclesShowCOM(TEXT("p.Vehicle.ShowCOM"), GVehicleDebugParams.ShowCOM, TEXT("Enable/Disable Center Of Mass Debug Visualisation."));
FAutoConsoleVariableRef CVarChaosVehiclesShowModelAxis(TEXT("p.Vehicle.ShowModelOrigin"), GVehicleDebugParams.ShowModelOrigin, TEXT("Enable/Disable Model Origin Visualisation."));
FAutoConsoleVariableRef CVarChaosVehiclesShowAllForces(TEXT("p.Vehicle.ShowAllForces"), GVehicleDebugParams.ShowAllForces, TEXT("Enable/Disable Force Visualisation."));
FAutoConsoleVariableRef CVarChaosVehiclesAerofoilForces(TEXT("p.Vehicle.ShowAerofoilForces"), GVehicleDebugParams.ShowAerofoilForces, TEXT("Enable/Disable Aerofoil Force Visualisation."));
FAutoConsoleVariableRef CVarChaosVehiclesAerofoilSurface(TEXT("p.Vehicle.ShowAerofoilSurface"), GVehicleDebugParams.ShowAerofoilSurface, TEXT("Enable/Disable a very approximate visualisation of where the Aerofoil surface is located and its orientation."));
FAutoConsoleVariableRef CVarChaosVehiclesDisableTorqueControl(TEXT("p.Vehicle.DisableTorqueControl"), GVehicleDebugParams.DisableTorqueControl, TEXT("Enable/Disable Direct Torque Control."));
FAutoConsoleVariableRef CVarChaosVehiclesDisableStabilizeControl(TEXT("p.Vehicle.DisableStabilizeControl"), GVehicleDebugParams.DisableStabilizeControl, TEXT("Enable/Disable Position Stabilization Control."));
FAutoConsoleVariableRef CVarChaosVehiclesDisableAerodynamics(TEXT("p.Vehicle.DisableAerodynamics"), GVehicleDebugParams.DisableAerodynamics, TEXT("Enable/Disable Aerodynamic Forces Drag/Downforce."));
FAutoConsoleVariableRef CVarChaosVehiclesDisableAerofoils(TEXT("p.Vehicle.DisableAerofoils"), GVehicleDebugParams.DisableAerofoils, TEXT("Enable/Disable Aerofoil Forces."));
FAutoConsoleVariableRef CVarChaosVehiclesDisableThrusters(TEXT("p.Vehicle.DisableThrusters"), GVehicleDebugParams.DisableThrusters, TEXT("Enable/Disable Thruster Forces."));
FAutoConsoleVariableRef CVarChaosVehiclesBatchQueries(TEXT("p.Vehicle.BatchQueries"), GVehicleDebugParams.BatchQueries, TEXT("Enable/Disable Batching Of Suspension Raycasts."));
FAutoConsoleVariableRef CVarChaosVehiclesCacheTraceOverlap(TEXT("p.Vehicle.CacheTraceOverlap"), GVehicleDebugParams.CacheTraceOverlap, TEXT("Enable/Disable Caching Of Suspension Trace Overlap Test Optimization (only valid when BatchQueries enabled)."));
FAutoConsoleVariableRef CVarChaosVehiclesForceDebugScaling(TEXT("p.Vehicle.SetForceDebugScaling"), GVehicleDebugParams.ForceDebugScaling, TEXT("Set Scaling For Force Visualisation."));
FAutoConsoleVariableRef CVarChaosVehiclesSleepCounterThreshold(TEXT("p.Vehicle.SleepCounterThreshold"), GVehicleDebugParams.SleepCounterThreshold, TEXT("Set The Sleep Counter Iteration Threshold."));
FAutoConsoleVariableRef CVarChaosVehiclesDisableVehicleSleep(TEXT("p.Vehicle.DisableVehicleSleep"), GVehicleDebugParams.DisableVehicleSleep, TEXT("Disable Vehicle Agressive Sleeping."));
FAutoConsoleVariableRef CVarChaosVehiclesSetMaxMPH(TEXT("p.Vehicle.SetMaxMPH"), GVehicleDebugParams.SetMaxMPH, TEXT("Set a top speed in MPH (affects all vehicles)."));
FAutoConsoleVariableRef CVarChaosVehiclesEnableMultithreading(TEXT("p.Vehicle.EnableMultithreading"), GVehicleDebugParams.EnableMultithreading, TEXT("Enable multi-threading of vehicle updates."));
FAutoConsoleVariableRef CVarChaosVehiclesControlInputWakeTolerance(TEXT("p.Vehicle.ControlInputWakeTolerance"), GVehicleDebugParams.ControlInputWakeTolerance, TEXT("Set the control input wake tolerance."));


void UChaosVehicleSimulation::ApplyDeferredForces(Chaos::FRigidBodyHandle_Internal* Handle)
{
	DeferredForces.Apply(Handle);
}

/** Pass control Input to the vehicle systems */
void UChaosVehicleSimulation::ApplyInput(const FControlInputs& ControlInputs, float DeltaTime)
{
	GetPVehicle()->ApplyAerofoils(ControlInputs.GetChaosInputs());
}


void UChaosVehicleSimulation::ApplyAerodynamics(float DeltaTime)
{
	if (!GVehicleDebugParams.DisableAerodynamics)
	{
		// This force applied all the time whether the vehicle is on the ground or not
		Chaos::FSimpleAerodynamicsSim& PAerodynamics = GetPVehicle()->GetAerodynamics();
		FVector LocalDragLiftForce = (PAerodynamics.GetCombinedForces(Chaos::CmToM(VehicleState.ForwardSpeed))) * Chaos::MToCmScaling();
		FVector WorldLiftDragForce = VehicleState.VehicleWorldTransform.TransformVector(LocalDragLiftForce);
		AddForce(WorldLiftDragForce);
	}
}

FVector GetWorldVelocityAtPoint(const Chaos::FRigidBodyHandle_Internal* RigidHandle, const FVector& WorldLocation)
{
	if (RigidHandle)
	{
		const Chaos::FVec3 COM = RigidHandle ? Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle) : (Chaos::FVec3)Chaos::FParticleUtilitiesGT::GetActorWorldTransform(RigidHandle).GetTranslation();
		const Chaos::FVec3 Diff = WorldLocation - COM;
		return RigidHandle->V() - Chaos::FVec3::CrossProduct(Diff, RigidHandle->W());
	}
	else
	{
		return FVector::ZeroVector;
	}
}

void UChaosVehicleSimulation::ApplyAerofoilForces(float DeltaTime)
{
	if (GVehicleDebugParams.DisableAerofoils || RigidHandle == nullptr)
		return;

	TArray<FVector> VelocityLocal;
	TArray<FVector> VelocityWorld;
	VelocityLocal.SetNum(GetPVehicle()->GetAerofoils().Num());
	VelocityWorld.SetNum(GetPVehicle()->GetAerofoils().Num());

	float Altitude = VehicleState.VehicleWorldTransform.GetLocation().Z;
	auto& aeros = GetPVehicle()->GetAerofoils();
	for (int AerofoilIdx = 0; AerofoilIdx < GetPVehicle()->GetAerofoils().Num(); AerofoilIdx++)
	{
		Chaos::FAerofoil& Aerofoil = aeros[AerofoilIdx];
		
		FVector WorldLocation = VehicleState.VehicleWorldTransform.TransformPosition(Aerofoil.Setup().Offset * Chaos::MToCmScaling());
		VelocityWorld[AerofoilIdx] = GetWorldVelocityAtPoint(RigidHandle, WorldLocation);
		VelocityLocal[AerofoilIdx] = VehicleState.VehicleWorldTransform.InverseTransformVector(VelocityWorld[AerofoilIdx]);
		

		FVector LocalForce = Aerofoil.GetForce(VehicleState.VehicleWorldTransform, VelocityLocal[AerofoilIdx] * Chaos::CmToMScaling(), Chaos::CmToM(Altitude), DeltaTime);

		FVector WorldForce = VehicleState.VehicleWorldTransform.TransformVector(LocalForce);
		WorldLocation = VehicleState.VehicleWorldTransform.TransformPosition(Aerofoil.GetCenterOfLiftOffset() * Chaos::MToCmScaling());
		AddForceAtPosition(WorldForce * Chaos::MToCmScaling(), WorldLocation);

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
		FVector WorldAxis = VehicleState.VehicleWorldTransform.TransformVector(FVector::CrossProduct(FVector(1, 0, 0), Aerofoil.Setup().UpAxis));
		if (GVehicleDebugParams.ShowAerofoilSurface)
		{
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(WorldLocation - WorldAxis * 150.0f, WorldLocation + WorldAxis * 150.0f, FColor::Black, false, -1.f, 0, 5.f);
		}
		if (GVehicleDebugParams.ShowAerofoilForces)
		{
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(WorldLocation, WorldLocation + WorldForce * GVehicleDebugParams.ForceDebugScaling, FColor::Green, false, -1.f, 0, 16.f);
		}
#endif
	}

}


void UChaosVehicleSimulation::ApplyThrustForces(float DeltaTime, const FControlInputs& ControlInputs)
{
	if (GVehicleDebugParams.DisableThrusters)
		return;

	for (auto& Thruster : GetPVehicle()->ApplyThrusters(ControlInputs.GetChaosInputs(),VehicleState,RigidHandle,DeltaTime)) {
		
		FVector COM_Offset = RigidHandle->CenterOfMass();
		COM_Offset.Z = 0.0f;
		FVector ThrustWorldLocation = VehicleState.VehicleWorldTransform.TransformPosition(Thruster.GetThrustLocation() + COM_Offset);
		FVector ThrustForce = VehicleState.VehicleWorldTransform.TransformPosition(Thruster.GetThrustForce());

		AddForceAtPosition(ThrustForce, ThrustWorldLocation);
	}
}


void UChaosVehicleSimulation::ApplyTorqueControl(float DeltaTime, const FControlInputs& InputData) {
	if (!GetPVehicle()->HasTorqueControlSetup())
	{
		return;
	}


	if (!GVehicleDebugParams.DisableTorqueControl)
	{
		AddTorqueInRadians(GetPVehicle()->ApplyTorqueControl(VehicleState,InputData.GetChaosInputs(),DeltaTime), true, true);
	}


	if (!GVehicleDebugParams.DisableStabilizeControl)
	{
		AddForce(GetPVehicle()->ApplyStabilizeControl(VehicleState,DeltaTime));
	}
}

void UChaosVehicleSimulation::DrawDebug3D()
{
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	if (RigidHandle == nullptr)
	{
		return;
	}

	const FTransform BodyTransform = VehicleState.VehicleWorldTransform;

	if (GVehicleDebugParams.ShowCOM)
	{
		const Chaos::FVec3 COMWorld = Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle);
		Chaos::FDebugDrawQueue::GetInstance().DrawDebugCoordinateSystem(COMWorld, FRotator(BodyTransform.GetRotation()), 200.f, false, -1.f, 0, 2.f);
	}

	if (GVehicleDebugParams.ShowModelOrigin)
	{
		Chaos::FDebugDrawQueue::GetInstance().DrawDebugCoordinateSystem(BodyTransform.GetLocation(), FRotator(BodyTransform.GetRotation()), 200.f, false, -1.f, 0, 2.f);
	}
#endif
}

void UChaosVehicleSimulation::AddForce(const FVector& Force, bool bAllowSubstepping, bool bAccelChange)
{
	DeferredForces.Add(FDeferredForces::FApplyForceData(Force, bAllowSubstepping, bAccelChange));

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	if (GVehicleDebugParams.ShowAllForces)
	{
		FVector Position = RigidHandle->X();
		Chaos::FDebugDrawQueue::GetInstance().DrawDebugDirectionalArrow(Position, Position + Force * GVehicleDebugParams.ForceDebugScaling
			, 20.f, FColor::Blue, false, 0, 0, 2.f);
	}
#endif	
}

void UChaosVehicleSimulation::AddForceAtPosition(const FVector& Force, const FVector& Position, bool bAllowSubstepping, bool bIsLocalForce)
{
	DeferredForces.Add(FDeferredForces::FApplyForceAtPositionData(Force, Position, bAllowSubstepping, bIsLocalForce));

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	if (GVehicleDebugParams.ShowAllForces)
	{
		Chaos::FDebugDrawQueue::GetInstance().DrawDebugDirectionalArrow(Position, Position + Force * GVehicleDebugParams.ForceDebugScaling
			, 20.f, FColor::Blue, false, 0, 0, 2.f);
	}
#endif
}

void UChaosVehicleSimulation::AddImpulse(const FVector& Impulse, bool bVelChange)
{
	DeferredForces.Add(FDeferredForces::FAddImpulseData(Impulse, bVelChange));

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	if (GVehicleDebugParams.ShowAllForces)
	{
		FVector Position = VehicleState.VehicleWorldCOM;
		Chaos::FDebugDrawQueue::GetInstance().DrawDebugDirectionalArrow(Position, Position + Impulse * GVehicleDebugParams.ForceDebugScaling
			, 20.f, FColor::Red, false, 0, 0, 2.f);
	}
#endif
	
}

void UChaosVehicleSimulation::AddImpulseAtPosition(const FVector& Impulse, const FVector& Position)
{
	DeferredForces.Add(FDeferredForces::FAddImpulseAtPositionData(Impulse, Position));

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	if (GVehicleDebugParams.ShowAllForces)
	{
		Chaos::FDebugDrawQueue::GetInstance().DrawDebugDirectionalArrow(Position, Position + Impulse * GVehicleDebugParams.ForceDebugScaling
			, 20.f, FColor::Red, false, 0, 0, 2.f);
	}
#endif

}

void UChaosVehicleSimulation::AddTorqueInRadians(const FVector& Torque, bool bAllowSubstepping /*= true*/, bool bAccelChange /*= false*/)
{
	DeferredForces.Add(FDeferredForces::FAddTorqueInRadiansData(Torque, bAllowSubstepping, bAccelChange));
}


/**
 * UChaosVehicleMovementComponent
 */
UChaosVehicleMovementComponent::UChaosVehicleMovementComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	bReverseAsBrake = true;
	Mass = 1500.0f;
	ChassisWidth = 180.f;
	ChassisHeight = 140.f;
	DragCoefficient = 0.3f;
	DownforceCoefficient = 0.3f;
	InertiaTensorScale = FVector( 1.0f, 1.0f, 1.0f );
	SleepThreshold = 10.0f;
	SleepSlopeLimit = 0.866f;	// 30 degrees, Cos(30)

	TorqueControl.InitDefaults();
	TargetRotationControl.InitDefaults();
	StabilizeControl.InitDefaults();

	AngErrorAccumulator = 0.0f;
	TargetGear = 0;

	PrevSteeringInput = 0.0f;
	PrevReplicatedSteeringInput = 0.0f;

	bRequiresControllerForInputs = true;
	IdleBrakeInput = 0.0f;
	StopThreshold = 10.0f; 
	WrongDirectionThreshold = 100.f;
	ThrottleInputRate.RiseRate = 6.0f;
	ThrottleInputRate.FallRate = 10.0f;
	ThrottleInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	BrakeInputRate.RiseRate = 6.0f;
	BrakeInputRate.FallRate = 10.0f;
	BrakeInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	SteeringInputRate.RiseRate = 2.5f;
	SteeringInputRate.FallRate = 5.0f;
	SteeringInputRate.InputCurveFunction = EInputFunctionType::SquaredFunction;
	HandbrakeInputRate.RiseRate = 12.0f;
	HandbrakeInputRate.FallRate = 12.0f;
	PitchInputRate.RiseRate = 6.0f;
	PitchInputRate.FallRate = 10.0f;
	PitchInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	RollInputRate.RiseRate = 6.0f;
	RollInputRate.FallRate = 10.0f;
	RollInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	YawInputRate.RiseRate = 6.0f;
	YawInputRate.FallRate = 10.0f;
	YawInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	TransmissionType = Chaos::ETransmissionType::Automatic;

	SetIsReplicatedByDefault(true);

	AHUD::OnShowDebugInfo.AddUObject(this, &UChaosVehicleMovementComponent::ShowDebugInfo);
}

// public

void UChaosVehicleMovementComponent::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);

	// Custom serialization goes here...
}

#if WITH_EDITOR
void UChaosVehicleMovementComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	// Trigger a runtime rebuild of the Chaos vehicle
	FChaosVehicleManager::VehicleSetupTag++;

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif // WITH_EDITOR

void UChaosVehicleMovementComponent::SetUpdatedComponent(USceneComponent* NewUpdatedComponent)
{
	//Skip PawnMovementComponent and simply set PawnOwner to null if we don't have a PawnActor as owner
	UNavMovementComponent::SetUpdatedComponent(NewUpdatedComponent);
	PawnOwner = NewUpdatedComponent ? Cast<APawn>(NewUpdatedComponent->GetOwner()) : nullptr;

	if(USkeletalMeshComponent* SKC = Cast<USkeletalMeshComponent>(NewUpdatedComponent))
	{
		SKC->bLocalSpaceKinematics = true;
	}
}

void UChaosVehicleMovementComponent::SetOverrideController(AController* InOverrideController)
{
	OverrideController = InOverrideController;
}


bool UChaosVehicleMovementComponent::ShouldCreatePhysicsState() const
{
	if (!IsRegistered() || IsBeingDestroyed())
	{
		return false;
	}

	// only create 'Physics' vehicle in game
	UWorld* World = GetWorld();
	if (World->IsGameWorld())
	{
		FPhysScene* PhysScene = World->GetPhysicsScene();

		if (PhysScene && FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene))
		{
			if (CanCreateVehicle())
			{
				return true;
			}
		}
	}
	return false;
}

void UChaosVehicleMovementComponent::NextDebugPage()
{
	int PageAsInt = (int)DebugPage;
	PageAsInt++;
	if (PageAsInt >= EDebugPages::MaxDebugPages)
	{
		PageAsInt = 0;
	}
	DebugPage = (EDebugPages)PageAsInt;
}

void UChaosVehicleMovementComponent::PrevDebugPage()
{
	int PageAsInt = (int)DebugPage;
	PageAsInt--;
	if (PageAsInt < 0)
	{
		PageAsInt = EDebugPages::MaxDebugPages - 1;
	}
	DebugPage = (EDebugPages)PageAsInt;
}

bool UChaosVehicleMovementComponent::HasValidPhysicsState() const
{
	return bPhysicsStateCreated;
}

bool UChaosVehicleMovementComponent::CanCreateVehicle() const
{
	check(GetOwner());
	FString ActorName = GetOwner()->GetName();

	if (UpdatedComponent == NULL)
	{
		UE_LOG(LogVehicle, Warning, TEXT("Can't create vehicle %s (%s). UpdatedComponent is not set."), *ActorName, *GetPathName());
		return false;
	}

	if (UpdatedPrimitive == NULL)
	{
		UE_LOG(LogVehicle, Warning, TEXT("Can't create vehicle %s (%s). UpdatedComponent is not a PrimitiveComponent."), *ActorName, *GetPathName());
		return false;
	}

	return true;
}


void UChaosVehicleMovementComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();
}

void UChaosVehicleMovementComponent::OnDestroyPhysicsState()
{
	if (HasValidPhysicsState())
	{
		if(auto* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(GetWorld()->GetPhysicsScene())) {
			VehicleManager->RemoveVehicle(this);
		}
		if (UpdatedComponent)
		{
			UpdatedComponent->RecreatePhysicsState();
		}
	}
	Super::OnDestroyPhysicsState();
}

void UChaosVehicleMovementComponent::PreTickGT(float DeltaTime)
{
	// movement updates and replication
	if (HasValidPhysicsState() && UpdatedComponent)
	{
		APawn* MyOwner = Cast<APawn>(UpdatedComponent->GetOwner());
		if (MyOwner)
		{
			UpdateState(DeltaTime);
		}
	}

	{
		// is this needless copying
		FControlInputs ControlInputs;
		ControlInputs.ThrottleInput = ThrottleInput;
		ControlInputs.BrakeInput = BrakeInput;
		ControlInputs.SteeringInput = SteeringInput;
		ControlInputs.HandbrakeInput = HandbrakeInput;
		ProcessSleeping(ControlInputs);
	}


	if (VehicleSetupTag != FChaosVehicleManager::VehicleSetupTag)
	{
		RecreatePhysicsState();
	}
}

void UChaosVehicleMovementComponent::StopMovementImmediately()
{
	FBodyInstance* TargetInstance = GetBodyInstance();
	if (TargetInstance)
	{
		// if start awake is false then setting the velocity (even to zero) causes particle to wake up.
		if (TargetInstance->IsInstanceAwake())
		{
			TargetInstance->SetLinearVelocity(FVector::ZeroVector, false);
			TargetInstance->SetAngularVelocityInRadians(FVector::ZeroVector, false);
			TargetInstance->ClearForces();
			TargetInstance->ClearTorques();
		}
	}
	Super::StopMovementImmediately();
	ClearAllInput();
}

// Input

void UChaosVehicleMovementComponent::SetThrottleInput(float Throttle)
{
	RawThrottleInput = FMath::Clamp(Throttle, -1.0f, 1.0f);
}

void UChaosVehicleMovementComponent::IncreaseThrottleInput(float ThrottleDelta)
{
	RawThrottleInput = FMath::Clamp(RawThrottleInput + ThrottleDelta, 0.f, 1.0f);
}

void UChaosVehicleMovementComponent::DecreaseThrottleInput(float ThrottleDelta)
{
	RawThrottleInput = FMath::Clamp(RawThrottleInput - ThrottleDelta, 0.f, 1.0f);
}

void UChaosVehicleMovementComponent::SetBrakeInput(float Brake)
{
	RawBrakeInput = FMath::Clamp(Brake, -1.0f, 1.0f);
}

void UChaosVehicleMovementComponent::SetSteeringInput(float Steering)
{
	RawSteeringInput = FMath::Clamp(Steering, -1.0f, 1.0f);
}

void UChaosVehicleMovementComponent::SetHandbrakeInput(bool bNewHandbrake)
{
	bRawHandbrakeInput = bNewHandbrake;
}

void UChaosVehicleMovementComponent::SetSleeping(bool bEnableSleep)
{
	if (bEnableSleep)
	{
		PutAllEnabledRigidBodiesToSleep();
		VehicleState.bSleeping = true;
	}
	else
	{
		WakeAllEnabledRigidBodies();
		VehicleState.bSleeping = false;
	}
}

void UChaosVehicleMovementComponent::SetChangeUpInput(bool bNewGearUp)
{
	bRawGearUpInput = bNewGearUp;
}

void UChaosVehicleMovementComponent::SetChangeDownInput(bool bNewGearDown)
{
	bRawGearDownInput = bNewGearDown;
}

void UChaosVehicleMovementComponent::SetTargetGear(int32 GearNum, bool bImmediate)
{
}

bool UChaosVehicleMovementComponent::IsOnAnySurface(const TArray<UPhysicalMaterial*> surfaces) {
	return true;
}

void UChaosVehicleMovementComponent::SetUseAutomaticGears(bool bUseAuto)
{
	TransmissionType = bUseAuto ? Chaos::ETransmissionType::Automatic : Chaos::ETransmissionType::Manual;
}

void UChaosVehicleMovementComponent::SetRequiresControllerForInputs(bool bRequiresController)
{
	bRequiresControllerForInputs = bRequiresController;
}

// Data access

int32 UChaosVehicleMovementComponent::GetCurrentGear() const
{
	return 0;
}

int32 UChaosVehicleMovementComponent::GetTargetGear() const
{
	return TargetGear;
}

bool UChaosVehicleMovementComponent::GetUseAutoGears() const
{
	return (TransmissionType == Chaos::ETransmissionType::Automatic);
}

float UChaosVehicleMovementComponent::GetForwardSpeed() const
{
	return VehicleState.ForwardSpeed;
}

float UChaosVehicleMovementComponent::GetForwardSpeedMPH() const
{
	return Chaos::CmSToMPH(GetForwardSpeed());
}

// input related
float UChaosVehicleMovementComponent::CalcSteeringInput()
{
	return RawSteeringInput;
}

void UChaosVehicleMovementComponent::CalcThrottleBrakeInput(float& ThrottleOut, float& BrakeOut)
{
	BrakeOut = RawBrakeInput;
	ThrottleOut = RawThrottleInput;

	if (bReverseAsBrake)
	{
		if (RawThrottleInput > 0.f)
		{
		// Note: Removed this condition to support wheel spinning when rolling backareds with accelerator pressed, rather than braking
		// Should this case be another checkbox option??
		//	
		//	// car moving backwards but player wants to move forwards...
		//	// if vehicle is moving backwards, then press brake
		//	if (VehicleState.ForwardSpeed < -WrongDirectionThreshold)
		//	{
		//		BrakeOut = 1.0f;
		//		ThrottleOut = 0.0f;
		//	}

		}
		else if (RawBrakeInput > 0.f)
		{
			// car moving forwards but player wants to move backwards...
			// if vehicle is moving forwards, then press brake
			if (VehicleState.ForwardSpeed > WrongDirectionThreshold)
			{
				BrakeOut = 1.0f;
				ThrottleOut = 0.0f;
			}
			else if (GetTargetGear() < 0)
			{
				ThrottleOut = RawBrakeInput;
				BrakeOut = 0.0f;
			}
		}
		// straight reversing
		else if (RawBrakeInput > 0.f && GetTargetGear() < 0)
		{
			ThrottleOut = RawBrakeInput;
		}
		else
		{
			// if player isn't pressing forward or backwards...
			if (VehicleState.ForwardSpeed < StopThreshold && VehicleState.ForwardSpeed > -StopThreshold)	//auto brake 
			{
				BrakeOut = 1.f;
			}
			else
			{
				BrakeOut = IdleBrakeInput;
			}
		}

		ThrottleOut = FMath::Clamp<float>(ThrottleOut, 0.0, 1.0);
		BrakeOut = FMath::Clamp<float>(BrakeOut, 0.0, 1.0);
	}
	else
	{
		BrakeOut = FMath::Abs(RawBrakeInput);

		// if player isn't pressing forward or backwards...
		if (RawBrakeInput < SMALL_NUMBER && RawThrottleInput < SMALL_NUMBER)
		{
			if (VehicleState.ForwardSpeed < StopThreshold && VehicleState.ForwardSpeed > -StopThreshold)	//auto brake 
			{
				BrakeOut = 1.f;
			}
		}

	}

}

bool UChaosVehicleMovementComponent::CalcHandbrakeInput()
{
	return bRawHandbrakeInput;
}

void UChaosVehicleMovementComponent::ClearInput()
{
	SteeringInput = 0.0f;
	ThrottleInput = 0.0f;
	BrakeInput = 0.0f;
	HandbrakeInput = 0.0f;

}

void UChaosVehicleMovementComponent::ClearRawInput()
{
	RawBrakeInput = 0.0f;
	RawSteeringInput = 0.0f;
	RawThrottleInput = 0.0f;
	bRawGearDownInput = false;
	bRawGearUpInput = false;
	bRawHandbrakeInput = false;
}

// Update

void UChaosVehicleMovementComponent::UpdateState(float DeltaTime)
{
	// update input values
	AController* Controller = GetController();
	VehicleState.CaptureState(GetBodyInstance(), GetGravityZ(), DeltaTime);
	VehicleState.NumWheelsOnGround = 0;
	VehicleState.bVehicleInAir = false;
	UpdateGroundedState(DeltaTime);
	bool bProcessLocally = bRequiresControllerForInputs?(Controller && Controller->IsLocalController()):true;

	// IsLocallyControlled will fail if the owner is unpossessed (i.e. Controller == nullptr);
	// Should we remove input instead of relying on replicated state in that case?
	if (bProcessLocally && HasValidPhysicsState())
	{
		if (bReverseAsBrake)
		{
			//for reverse as state we want to automatically shift between reverse and first gear
			// Note: Removed this condition to support wheel spinning when rolling backwards with accelerator pressed, rather than braking
			//if (FMath::Abs(GetForwardSpeed()) < WrongDirectionThreshold)	//we only shift between reverse and first if the car is slow enough.
			{
				if (RawBrakeInput > KINDA_SMALL_NUMBER && GetCurrentGear() >= 0 && GetTargetGear() >= 0)
				{
					SetTargetGear(-1, true);
				}
				else if (RawThrottleInput > KINDA_SMALL_NUMBER && GetCurrentGear() <= 0 && GetTargetGear() <= 0)
				{
					SetTargetGear(1, true);
				}
			}
		}
		else
		{
			if (TransmissionType == Chaos::ETransmissionType::Automatic)
			{
				if (RawThrottleInput > KINDA_SMALL_NUMBER
					&& GetCurrentGear() == 0
					&& GetTargetGear() == 0)
				{
					SetTargetGear(1, true);
				}
			}

		}

		float ModifiedThrottle = 0.f;
		float ModifiedBrake = 0.f;
		CalcThrottleBrakeInput(ModifiedThrottle, ModifiedBrake);
		SteeringInput = SteeringInputRate.InterpInputValue(DeltaTime, SteeringInput, CalcSteeringInput());
		ThrottleInput = ThrottleInputRate.InterpInputValue(DeltaTime, ThrottleInput, ModifiedThrottle);
		BrakeInput = BrakeInputRate.InterpInputValue(DeltaTime, BrakeInput, ModifiedBrake);
		HandbrakeInput = CalcHandbrakeInput();

		// and send to server - (ServerUpdateState_Implementation below)
		ServerUpdateState(SteeringInput, ThrottleInput, BrakeInput, HandbrakeInput, GetCurrentGear());

		if (PawnOwner && PawnOwner->IsNetMode(NM_Client))
		{
			MarkForClientCameraUpdate();
		}
	}
	else
	{
		// use replicated values for remote pawns
		SteeringInput = ReplicatedState.SteeringInput;
		ThrottleInput = ReplicatedState.ThrottleInput;
		BrakeInput = ReplicatedState.BrakeInput;
		HandbrakeInput = ReplicatedState.HandbrakeInput;
		SetTargetGear(ReplicatedState.TargetGear, true);
	}
}


void UChaosVehicleMovementComponent::ProcessSleeping(const FControlInputs& ControlInputs)
{
	FBodyInstance* TargetInstance = GetBodyInstance();
	if (TargetInstance)
	{
		bool PrevSleeping = VehicleState.bSleeping;
		VehicleState.bSleeping = !TargetInstance->IsInstanceAwake();

		// The physics system has woken vehicle up due to a collision or something
		if (PrevSleeping && !VehicleState.bSleeping)
		{
			VehicleState.SleepCounter = 0;
		}

		// If the vehicle is locally controlled, we want to use the raw inputs to determine sleep.
		// However, if it's on the Server or is just being replicated to other Clients then there
		// won't be any Raw input. In that case, use ReplicatedState instead.
		
		// NOTE: Even on local clients, ReplicatedState will still be populated (the call to ServerUpdateState will
		//			be processed locally). Maybe we should *just* use ReplicatedState?

		const AController* Controller = GetController();
		const bool bIsLocallyControlled = (Controller && Controller->IsLocalController());
		const bool bControlInputPressed = bIsLocallyControlled ? (ControlInputs.ThrottleInput >= GVehicleDebugParams.ControlInputWakeTolerance) || (FMath::Abs(ControlInputs.SteeringInput - PrevSteeringInput) >= GVehicleDebugParams.ControlInputWakeTolerance)
			: (ReplicatedState.ThrottleInput >= GVehicleDebugParams.ControlInputWakeTolerance) || (FMath::Abs(ReplicatedState.SteeringInput - PrevReplicatedSteeringInput) >= GVehicleDebugParams.ControlInputWakeTolerance);

		PrevSteeringInput = ControlInputs.SteeringInput;
		PrevReplicatedSteeringInput = ReplicatedState.SteeringInput;

		// Wake if control input pressed
		if ((VehicleState.bSleeping && bControlInputPressed) || GVehicleDebugParams.DisableVehicleSleep)
		{
			VehicleState.bSleeping = false;
			VehicleState.SleepCounter = 0;
			SetSleeping(false);
		}
		else if (!GVehicleDebugParams.DisableVehicleSleep && !VehicleState.bSleeping && !bControlInputPressed && VehicleState.bAllWheelsOnGround && (VehicleState.VehicleUpAxis.Z > SleepSlopeLimit))
		{
			float SpeedSqr = TargetInstance->GetUnrealWorldVelocity().SizeSquared();
			if (SpeedSqr < (SleepThreshold* SleepThreshold))
			{
				if (VehicleState.SleepCounter < GVehicleDebugParams.SleepCounterThreshold)
				{
					VehicleState.SleepCounter++;
				}
				else
				{
					VehicleState.bSleeping = true;
					SetSleeping(true);
				}
			}
		}
	}
}

/// @cond DOXYGEN_WARNINGS

bool UChaosVehicleMovementComponent::ServerUpdateState_Validate(float InSteeringInput, float InThrottleInput, float InBrakeInput, bool InHandbrakeInput, int32 InCurrentGear)
{
	return true;
}

void UChaosVehicleMovementComponent::ServerUpdateState_Implementation(float InSteeringInput, float InThrottleInput, float InBrakeInput
	, bool InHandbrakeInput, int32 InCurrentGear)
{
	SteeringInput = InSteeringInput;
	ThrottleInput = InThrottleInput;
	BrakeInput = InBrakeInput;
	HandbrakeInput = InHandbrakeInput;

	if (!GetUseAutoGears())
	{
		if (UWorld* World = GetWorld())
		{
			if (World->GetNetMode() == NM_DedicatedServer)
			{
				SetTargetGear(InCurrentGear, true);
			}
		}
	}

	// update state of inputs
	ReplicatedState.SteeringInput = InSteeringInput;
	ReplicatedState.ThrottleInput = InThrottleInput;
	ReplicatedState.BrakeInput = InBrakeInput;
	ReplicatedState.HandbrakeInput = InHandbrakeInput;
	ReplicatedState.TargetGear = InCurrentGear;

}

/// @endcond


// Setup
AController* UChaosVehicleMovementComponent::GetController() const
{
	if (OverrideController)
	{
		return OverrideController;
	}

	if (UpdatedComponent)
	{
		if (APawn* Pawn = Cast<APawn>(UpdatedComponent->GetOwner()))
		{
			return Pawn->Controller;
		}
	}

	return nullptr;
}


FBodyInstance* UChaosVehicleMovementComponent::GetBodyInstance()
{
	return UpdatedPrimitive ? UpdatedPrimitive->GetBodyInstance() : nullptr;
}

const FBodyInstance* UChaosVehicleMovementComponent::GetBodyInstance() const
{
	return UpdatedPrimitive ? UpdatedPrimitive->GetBodyInstance() : nullptr;
}

UMeshComponent* UChaosVehicleMovementComponent::GetMesh() const
{
	return Cast<UMeshComponent>(UpdatedComponent);
}

USkeletalMeshComponent* UChaosVehicleMovementComponent::GetSkeletalMesh()
{
	return Cast<USkeletalMeshComponent>(UpdatedComponent);
}

UStaticMeshComponent* UChaosVehicleMovementComponent::GetStaticMesh()
{
	return Cast<UStaticMeshComponent>(UpdatedComponent);
}

FVector UChaosVehicleMovementComponent::LocateBoneOffset(const FName InBoneName, const FVector& InExtraOffset) const
{
	FVector Offset = InExtraOffset;

	if (InBoneName != NAME_None)
	{
		if (USkinnedMeshComponent* Mesh = Cast<USkinnedMeshComponent>(GetMesh()))
		{
			if (ensureMsgf(Mesh->GetSkinnedAsset(), TEXT("Expected skinned asset when locating bone offset. Asset might be missing references.")))
			{
				const FVector BonePosition = Mesh->GetSkinnedAsset()->GetComposedRefPoseMatrix(InBoneName).GetOrigin() * Mesh->GetRelativeScale3D();
				//BonePosition is local for the root BONE of the skeletal mesh - however, we are using the Root BODY which may have its own transform, so we need to return the position local to the root BODY
				FMatrix RootBodyMTX = FMatrix::Identity;

				if (Mesh->GetBodyInstance() && Mesh->GetBodyInstance()->BodySetup.IsValid())
				{
					RootBodyMTX = Mesh->GetSkinnedAsset()->GetComposedRefPoseMatrix(Mesh->GetBodyInstance()->BodySetup->BoneName);
				}
				const FVector LocalBonePosition = RootBodyMTX.InverseTransformPosition(BonePosition);
				Offset += LocalBonePosition;
			}
		}
	}
	return Offset;
}

void UChaosVehicleMovementComponent::CreateVehicle() {
}

void UChaosVehicleMovementComponent::PostSetupVehicle()
{
}

void UChaosVehicleMovementComponent::SetupVehicleMass()
{
	if (UpdatedPrimitive && UpdatedPrimitive->GetBodyInstance())
	{
		//Ensure that if mass properties ever change we set them back to our override
		UpdatedPrimitive->GetBodyInstance()->OnRecalculatedMassProperties().AddUObject(this, &UChaosVehicleMovementComponent::UpdateMassProperties);

		UpdateMassProperties(UpdatedPrimitive->GetBodyInstance());
	}
}

void UChaosVehicleMovementComponent::UpdateMassProperties(FBodyInstance* BodyInstance)
{
	if (BodyInstance && FPhysicsInterface::IsValid(BodyInstance->ActorHandle) && FPhysicsInterface::IsRigidBody(BodyInstance->ActorHandle))
	{
		FPhysicsCommand::ExecuteWrite(BodyInstance->ActorHandle, [&](FPhysicsActorHandle& Actor)
			{
				const float MassRatio = this->Mass > 0.0f ? this->Mass / BodyInstance->GetBodyMass() : 1.0f;

				FVector InertiaTensor = BodyInstance->GetBodyInertiaTensor();

				InertiaTensor.X *= this->InertiaTensorScale.X * MassRatio;
				InertiaTensor.Y *= this->InertiaTensorScale.Y * MassRatio;
				InertiaTensor.Z *= this->InertiaTensorScale.Z * MassRatio;

				if (bEnableCenterOfMassOverride)
				{
					FTransform COMTransform = FPhysicsInterface::GetComTransformLocal_AssumesLocked(Actor);
					COMTransform.SetTranslation(CenterOfMassOverride + BodyInstance->COMNudge);
					FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor, COMTransform);
				}
				FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor, InertiaTensor);
				FPhysicsInterface::SetMass_AssumesLocked(Actor, this->Mass);
			});
	}

}

void UChaosVehicleMovementComponent::ComputeConstants()
{
	DragArea = ChassisWidth * ChassisHeight;
}


// Debug
void UChaosVehicleMovementComponent::ShowDebugInfo(AHUD* HUD, UCanvas* Canvas, const FDebugDisplayInfo& DisplayInfo, float& YL, float& YPos)
{
	static FName NAME_Vehicle = FName(TEXT("Vehicle"));

	if (Canvas && HUD->ShouldDisplayDebug(NAME_Vehicle))
	{
		if (APlayerController* Controller = Cast<APlayerController>(GetController()))
		{
			if (Controller->IsLocalController())
			{
				DrawDebug(Canvas, YL, YPos);
			}
		}
	}
}

void UChaosVehicleMovementComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	FBodyInstance* TargetInstance = GetBodyInstance();
	if (HasValidPhysicsState() || TargetInstance == nullptr)
	{
		return;
	}

	UFont* RenderFont = GEngine->GetMediumFont();
	// draw general vehicle data
	{
		Canvas->SetDrawColor(FColor::White);
		YPos += 16;

		float ForwardSpeedKmH = Chaos::CmSToKmH(GetForwardSpeed());
		float ForwardSpeedMPH = Chaos::CmSToMPH(GetForwardSpeed());
		float ForwardSpeedMSec = Chaos::CmToM(GetForwardSpeed());

		if (TargetInstance)
		{
			FVector FinalCOM = TargetInstance->GetMassSpaceLocal().GetTranslation();
			FVector OffsetCOM = TargetInstance->COMNudge;
			FVector BaseCOM = FinalCOM - TargetInstance->COMNudge;
			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Mass (Kg): %.1f"), TargetInstance->GetBodyMass()), 4, YPos);
			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Local COM : %s"), *FinalCOM.ToString()), 4, YPos);
			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("[COM Base : %s  COM Offset : %s]"), *BaseCOM.ToString(), *OffsetCOM.ToString()), 4, YPos);
			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Inertia : %s"), *TargetInstance->GetBodyInertiaTensor().ToString()), 4, YPos);
		}

		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Awake %d (Vehicle Sleep %d)"), TargetInstance->IsInstanceAwake(), VehicleState.bSleeping), 4, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Speed (km/h): %.1f  (MPH): %.1f  (m/s): %.1f"), ForwardSpeedKmH, ForwardSpeedMPH, ForwardSpeedMSec), 4, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Acceleration (m/s-2): %.1f"), Chaos::CmToM(VehicleState.LocalAcceleration.X)), 4, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("GForce : %2.1f"), VehicleState.LocalGForce.X), 4, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Steering: %.1f (RAW %.1f)"), SteeringInput, RawSteeringInput), 4, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Throttle: %.1f (RAW %.1f)"), ThrottleInput, RawThrottleInput), 4, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Brake: %.1f (RAW %.1f)"), BrakeInput, RawBrakeInput), 4, YPos);
		FString GearState = GetUseAutoGears() ? "Automatic" : "Manual";
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Gears: %s"), *GearState), 4, YPos);
	}

#endif
}

/// @cond DOXYGEN_WARNINGS

void UChaosVehicleMovementComponent::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
	Super::GetLifetimeReplicatedProps( OutLifetimeProps );

	DOREPLIFETIME( UChaosVehicleMovementComponent, ReplicatedState );
	DOREPLIFETIME(UChaosVehicleMovementComponent, OverrideController);
}

/// @endcond


void UChaosVehicleMovementComponent::DrawLine2D(UCanvas* Canvas, const FVector2D& StartPos, const FVector2D& EndPos, FColor Color, float Thickness)
{
	if (Canvas)
	{
		FCanvasLineItem LineItem(StartPos, EndPos);
		LineItem.SetColor(Color);
		LineItem.LineThickness = Thickness;
		Canvas->DrawItem(LineItem);
	}
}


void FVehicleAerofoilConfig::FillAerofoilSetup(const UChaosVehicleMovementComponent& MovementComponent)
{
	PAerofoilConfig.Type = (Chaos::EAerofoilType)(this->AerofoilType);
	PAerofoilConfig.Offset = MovementComponent.LocateBoneOffset(this->BoneName, this->Offset);
	PAerofoilConfig.UpAxis = this->UpAxis;
	PAerofoilConfig.Area = this->Area;
	PAerofoilConfig.Camber = this->Camber;
	PAerofoilConfig.MaxControlAngle = this->MaxControlAngle;
	PAerofoilConfig.StallAngle = this->StallAngle;
	PAerofoilConfig.LiftMultiplier = this->LiftMultiplier;
	PAerofoilConfig.DragMultiplier = this->DragMultiplier;
}

void FVehicleThrustConfig::FillThrusterSetup(const UChaosVehicleMovementComponent& MovementComponent)
{
	PThrusterConfig.Type = (Chaos::EThrustType)(this->ThrustType);
	PThrusterConfig.Offset = MovementComponent.LocateBoneOffset(this->BoneName, this->Offset);
	PThrusterConfig.Axis = this->ThrustAxis;
	//	PThrusterConfig.ThrustCurve = this->ThrustCurve;
	PThrusterConfig.MaxThrustForce = Chaos::MToCm(this->MaxThrustForce);
	PThrusterConfig.MaxControlAngle = this->MaxControlAngle;
}


// ---- ASYNC ----

/************************************************************************/
/* PASS ANY INUTS TO THE PHYSICS THREAD SIMULATION IN HERE              */
/************************************************************************/
void UChaosVehicleMovementComponent::Update(float DeltaTime)
{
	
}

void UChaosVehicleMovementComponent::ResetVehicleState()
{
	ClearRawInput();
	StopMovementImmediately();

	OnDestroyPhysicsState();
	OnCreatePhysicsState();

	// Shift into neutral, force the local copy of target gear to be correct
	SetTargetGear(0, true);
	TargetGear = 0;
}


/***************************************************************************/
/* READ OUTPUT DATA - Access the async output data from the Physics Thread */
/***************************************************************************/
void UChaosVehicleMovementComponent::ParallelUpdate(float DeltaSeconds)
{
}

void UChaosVehicleMovementComponent::GetBaseSnapshot(FBaseSnapshotData& SnapshotOut) const
{
	if (const FBodyInstance* TargetInstance = GetBodyInstance())
	{
		SnapshotOut.Transform = TargetInstance->GetUnrealWorldTransform();
		SnapshotOut.LinearVelocity = TargetInstance->GetUnrealWorldVelocity();
		SnapshotOut.AngularVelocity = TargetInstance->GetUnrealWorldAngularVelocityInRadians();
	}
}

void UChaosVehicleMovementComponent::SetBaseSnapshot(const FBaseSnapshotData& SnapshotIn)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		TargetInstance->SetLinearVelocity(SnapshotIn.LinearVelocity, false);
		TargetInstance->SetAngularVelocityInRadians(SnapshotIn.AngularVelocity, false);
		TargetInstance->SetBodyTransform(SnapshotIn.Transform, ETeleportType::TeleportPhysics);
	}
}

void UChaosVehicleMovementComponent::WakeAllEnabledRigidBodies()
{
	if (USkeletalMeshComponent* Mesh = GetSkeletalMesh())
	{
		for (int32 i = 0; i < Mesh->Bodies.Num(); i++)
		{
			FBodyInstance* BI = Mesh->Bodies[i];
			check(BI);

			if (!BI->IsPhysicsDisabled() && BI->IsNonKinematic())
			{
				BI->WakeInstance();
			}
		}
	}
}

void UChaosVehicleMovementComponent::PutAllEnabledRigidBodiesToSleep()
{
	if (USkeletalMeshComponent* Mesh = GetSkeletalMesh())
	{
		for (int32 i = 0; i < Mesh->Bodies.Num(); i++)
		{
			FBodyInstance* BI = Mesh->Bodies[i];
			check(BI);

			if (!BI->IsPhysicsDisabled() && BI->IsNonKinematic())
			{
				BI->PutInstanceToSleep();
			}
		}
	}
}


#undef LOCTEXT_NAMESPACE


#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif


