// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SnapshotData.generated.h"


USTRUCT(BlueprintType)
struct VEHICLES_API FBaseSnapshotData //: public UObject
{
public:
	GENERATED_BODY()

	FBaseSnapshotData()
	{
		Transform = FTransform::Identity;
		LinearVelocity = FVector::ZeroVector;
		AngularVelocity = FVector::ZeroVector;
	}

	UPROPERTY()
	FTransform Transform;		// world coords

	UPROPERTY()
	FVector LinearVelocity;		// world coords

	UPROPERTY()
	FVector AngularVelocity;	// world coords

};