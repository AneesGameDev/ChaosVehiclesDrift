// Copyright Epic Games, Inc. All Rights Reserved.

#include "HoveringVehiclePawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "ChaosVehicleMovementComponent.h"
#include "ChaosHoveringVehicleMovementComponent.h"
#include "DisplayDebugHelpers.h"

AHoveringVehiclePawn::AHoveringVehiclePawn(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	VehicleMovementComponent = CreateDefaultSubobject<UChaosHoveringVehicleMovementComponent>(VehicleMovementComponentName);
	VehicleMovementComponent->SetIsReplicated(true); // Enable replication by default
	VehicleMovementComponent->UpdatedComponent = GetMesh();
}

const bool Chaos::FSimpleHoveringVehicle::IsGrounded(int MoverIndex) {
	if(MoverIndex > -1)
		return Elevators[MoverIndex].InContact();

	bool grounded = false;
	for (auto& e : Elevators) {
		grounded |= e.InContact();
	}
	return grounded;
}

UChaosHoveringVehicleMovementComponent* AHoveringVehiclePawn::GetHoveringVehicleMovement() const {
	return VehicleMovementComponent;
}


class UChaosVehicleMovementComponent* AHoveringVehiclePawn::GetVehicleMovementComponent() const
{
	return VehicleMovementComponent;
}

