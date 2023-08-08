// Copyright Epic Games, Inc. All Rights Reserved.

#include "WheeledVehiclePawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "ChaosVehicleMovementComponent.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "DisplayDebugHelpers.h"

AWheeledVehiclePawn::AWheeledVehiclePawn(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	VehicleMovementComponent = CreateDefaultSubobject<UChaosWheeledVehicleMovementComponent>(VehicleMovementComponentName);
	VehicleMovementComponent->SetIsReplicated(true); // Enable replication by default
	VehicleMovementComponent->UpdatedComponent = GetMesh();
}

UChaosWheeledVehicleMovementComponent* AWheeledVehiclePawn::GetWheeledVehicleMovement() const {
	return VehicleMovementComponent;
}

const bool Chaos::FSimpleWheeledVehicle::IsGrounded(int MoverIndex) {
	return Wheels[MoverIndex].InContact();
}


class UChaosVehicleMovementComponent* AWheeledVehiclePawn::GetVehicleMovementComponent() const
{
	return VehicleMovementComponent;
}

