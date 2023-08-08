// Copyright Epic Games, Inc. All Rights Reserved.

#include "ChaosVehicleElevator.h"

#include "ChaosHoveringVehicleManagerAsyncCallback.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Vehicles/TireType.h"
#include "GameFramework/PawnMovementComponent.h"
#include "ChaosVehicleManager.h"
#include "ChaosHoveringVehicleMovementComponent.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif


UChaosVehicleElevator::UChaosVehicleElevator(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CollisionMeshObj(TEXT("/Engine/EngineMeshes/Cylinder"));
	CollisionMesh = CollisionMeshObj.Object;

	WheelRadius = 32.0f;

	SpringRate = 250.0f;
	SpringPreload = 50.f;
	SuspensionAxis = FVector(0.f, 0.f, -1.f);
	SuspensionForceOffset = FVector::ZeroVector;
	SuspensionMaxRaise = 10.0f;
	SuspensionMaxDrop = 10.0f;
	SuspensionDampingRatio = 0.5f;
	SuspensionSmoothing = 0;
	WheelLoadRatio = 0.5f;
	RollbarScaling = 0.15f;
	SweepType = ESweepType::SimpleSweep;
}


FChaosVehicleManager* UChaosVehicleElevator::GetVehicleManager() const
{
	UWorld* World = GEngine->GetWorldFromContextObject(VehicleComponent, EGetWorldErrorMode::LogAndReturnNull);
	return World ? FChaosVehicleManager::GetVehicleManagerFromScene(World->GetPhysicsScene()) : nullptr;
}


float UChaosVehicleElevator::GetRotationAngle() const
{
	check(VehicleComponent && VehicleComponent->PVehicleOutput);
	float RotationAngle = -1.0f * FMath::RadiansToDegrees(VehicleComponent->PVehicleOutput->elevators[WheelIndex].AngularPosition);
	ensure(!FMath::IsNaN(RotationAngle));
	return RotationAngle;
}

float UChaosVehicleElevator::GetRotationAngularVelocity() const
{
	check(VehicleComponent && VehicleComponent->PVehicleOutput);
	float RotationAngularVelocity = -1.0f * FMath::RadiansToDegrees(VehicleComponent->PVehicleOutput->elevators[WheelIndex].AngularVelocity);
	ensure(!FMath::IsNaN(RotationAngularVelocity));
	return RotationAngularVelocity;
}


float UChaosVehicleElevator::GetWheelRadius() const
{
	check(VehicleComponent && VehicleComponent->PVehicleOutput);
	return VehicleComponent->PVehicleOutput->elevators[WheelIndex].WheelRadius;
}

float UChaosVehicleElevator::GetWheelAngularVelocity() const
{
	check(VehicleComponent && VehicleComponent->PVehicleOutput);
	return VehicleComponent->PVehicleOutput->elevators[WheelIndex].AngularVelocity;
}

float UChaosVehicleElevator::GetSuspensionOffset() const
{
	check(VehicleComponent && VehicleComponent->PVehicleOutput);
	return VehicleComponent->GetSuspensionOffset(WheelIndex);
}

FVector UChaosVehicleElevator::GetSuspensionAxis() const
{
	check(VehicleComponent);
	return SuspensionAxis;
}

bool UChaosVehicleElevator::IsInAir() const
{
	check(VehicleComponent && VehicleComponent->PVehicleOutput);
	return !VehicleComponent->PVehicleOutput->elevators[WheelIndex].InContact;
}


void UChaosVehicleElevator::Init( UChaosHoveringVehicleMovementComponent* InVehicleSim, int32 InWheelIndex )
{
	check(InVehicleSim);
	check(InVehicleSim->Wheels.IsValidIndex(InWheelIndex));

	VehicleComponent = InVehicleSim;
	WheelIndex = InWheelIndex;

	Location = GetPhysicsLocation();
	OldLocation = Location;
}

void UChaosVehicleElevator::Shutdown()
{
//	WheelShape = NULL;
}

FChaosElevatorSetup& UChaosVehicleElevator::GetElevatorSetup()
{
	return VehicleComponent->WheelSetups[WheelIndex];
}

void UChaosVehicleElevator::Tick( float DeltaTime )
{
	OldLocation = Location;
	Location = GetPhysicsLocation();
	Velocity = ( Location - OldLocation ) / DeltaTime;
}

FVector UChaosVehicleElevator::GetPhysicsLocation()
{
	return Location;
}

#if WITH_EDITOR

void UChaosVehicleElevator::PostEditChangeProperty( FPropertyChangedEvent& PropertyChangedEvent )
{
	// Trigger a runtime rebuild of the Physics vehicle
	FChaosVehicleManager::VehicleSetupTag++;

	Super::PostEditChangeProperty(PropertyChangedEvent);
}

#endif //WITH_EDITOR


UPhysicalMaterial* UChaosVehicleElevator::GetContactSurfaceMaterial()
{
	check(VehicleComponent && VehicleComponent->PVehicleOutput);
	return VehicleComponent->GetPhysMaterial(WheelIndex);
}


#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif



