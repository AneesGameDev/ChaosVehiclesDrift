// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SnapshotData.h"
#include "HoveringSnapshotData.generated.h"


USTRUCT(BlueprintType)
struct HOVERINGVEHICLES_API FElevatorSnapshot
{
	GENERATED_BODY()

	FElevatorSnapshot()
	{
		SuspensionOffset = 0.f;
		WheelRotationAngle = 0.f;
		WheelRadius = 0.f;
		WheelAngularVelocity = 0.f;
	}
	
	UPROPERTY()
	float SuspensionOffset;		// suspension location
	
	UPROPERTY()
	float WheelRotationAngle;	// wheel rotation angle, rotated position
		// steering position

	UPROPERTY()
	float WheelRadius;			// radius of the wheel can be changed dynamically, to sim damaged ot flat

	UPROPERTY()
	float WheelAngularVelocity;	// speed of rotation of wheel
};

USTRUCT(BlueprintType)
struct HOVERINGVEHICLES_API FHoveringSnapshotData : public FBaseSnapshotData
{
public:
	GENERATED_BODY()

	FHoveringSnapshotData()
	{
		SelectedGear = 0;
		EngineRPM = 0.f;
	}

	UPROPERTY()
	int SelectedGear;		// -ve reverse gear(s), 0 neutral, +ve forward gears

	UPROPERTY()
	float EngineRPM;		// Engine Revolutions Per Minute

	UPROPERTY()
	TArray<FElevatorSnapshot> WheelSnapshots;

};
