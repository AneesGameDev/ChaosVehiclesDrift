// Copyright Epic Games, Inc. All Rights Reserved.

#include "BaseVehiclePawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "DisplayDebugHelpers.h"

FName ABaseVehiclePawn::VehicleMovementComponentName(TEXT("VehicleMovementComp"));
FName ABaseVehiclePawn::VehicleMeshComponentName(TEXT("VehicleMesh"));

ABaseVehiclePawn::ABaseVehiclePawn(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	Mesh = CreateDefaultSubobject<USkeletalMeshComponent>(VehicleMeshComponentName);
	Mesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
	Mesh->BodyInstance.bSimulatePhysics = false;
	Mesh->BodyInstance.bNotifyRigidBodyCollision = true;
	Mesh->BodyInstance.bUseCCD = true;
	Mesh->bBlendPhysics = true;
	Mesh->SetGenerateOverlapEvents(true);
	Mesh->SetCanEverAffectNavigation(false);
	RootComponent = Mesh;
}

void ABaseVehiclePawn::DisplayDebug(UCanvas* Canvas, const FDebugDisplayInfo& DebugDisplay, float& YL, float& YPos)
{
	static FName NAME_Vehicle = FName(TEXT("Vehicle"));

	Super::DisplayDebug(Canvas, DebugDisplay, YL, YPos);
}
