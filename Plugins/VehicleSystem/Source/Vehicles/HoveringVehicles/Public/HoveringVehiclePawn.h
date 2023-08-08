// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BaseVehiclePawn.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Pawn.h"
#include "HoveringVehiclePawn.generated.h"

class FDebugDisplayInfo;
/**
 * ChaosWheeledVehicle is the base wheeled vehicle pawn actor.
 * By default it uses UChaosHoveringVehicleMovementComponent for its simulation, but this can be overridden by inheriting from the class and modifying its constructor like so:
 * Super(ObjectInitializer.SetDefaultSubobjectClass<UMyMovement>(VehicleComponentName))
 * Where UMyMovement is the new movement type that inherits from UChaosVehicleMovementComponent
 */

UCLASS(abstract, config=Game, BlueprintType)
class HOVERINGVEHICLES_API AHoveringVehiclePawn : public ABaseVehiclePawn
{
	GENERATED_BODY()

	AHoveringVehiclePawn(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());
	/** vehicle simulation component */
	UPROPERTY(Category = Vehicle, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	TObjectPtr<class UChaosHoveringVehicleMovementComponent> VehicleMovementComponent;
public:
	/** Util to get the wheeled vehicle movement component */
	virtual class UChaosVehicleMovementComponent* GetVehicleMovementComponent() const override;

	/** Returns VehicleMovement subobject **/
	class UChaosHoveringVehicleMovementComponent* GetHoveringVehicleMovement() const;
};
