// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Containers/Array.h"
#include "Math/UnrealMathSSE.h"
#include "Math/Vector.h"
#include "TireSystem.h"
#include "VehicleSystemTemplate.h"
#include "VehicleUtility.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

namespace Chaos
{

/**
 * Tire load changes, when cornering outer tires are loaded and inner ones unloaded
 * Similarly load changes when accelerating and breaking.
 * Fx : brake/drive force
 * Fy : Cornering Force
 * Fz : Tire load (vehicle weight)
 *
 * Mx : overturning moment
 * My : moment about brake/drive torque
 * Mz : self-aligning moment
 *
 * Fx : forward speed at wheel center
 *
 *
 * Omega : Rotational Speed [radians/sec]
 * Alpha : Slip Angle [radians]
 * k or Sx: Longitudinal Slip - slip is -ve when braking and +ve when accelerating
 * Re : Effective Wheel Radius
 */

/**
* Wheel setup data that doesn't change during simulation
*/
struct VEHICLEPARTS_API FSimpleElevatorConfig
{
	// #todo: use this
	enum EWheelDamageStatus
	{
		NONE,
		BUCKLED,
		FLAT,
		MISSING
	};

	// #todo: use this
	enum EWheelSimulationStatus
	{
		ROLLING,	// wheel speed matches the vehicle ground speed
		SPINNING,	// wheel speed faster than vehicle ground speed
		LOCKED		// wheel is locked and sliding over surface
	};

	// #todo: use this
	enum EFrictionCombineMethod
	{
		Multiply, // default - most correct
		Average
	};

	enum EAxleType : uint8
	{
		UndefinedAxle = 0,
		Front,
		Rear
	};
	
	enum EExternalTorqueCombineMethod : uint8
	{
		None = 0,
		Override,
		Additive
	};

	FSimpleElevatorConfig() 
		: Offset(FVector(2.f, 1.f, 0.f))
		, WheelMass(20.f) // [Kg]
		, WheelRadius(30.f) // [cm]
		, AxleType(EAxleType::UndefinedAxle)
		, FrictionCombineMethod(EFrictionCombineMethod::Multiply)
		, ExternalTorqueCombineMethod(EExternalTorqueCombineMethod::None)
		, FrictionMultiplier(2.0f)
		, LateralSlipGraphMultiplier(1.0f)
		, CorneringStiffness(1000.0f)
		, SideSlipModifier(1.0f)
		, SlipThreshold(20.0f)
		, SkidThreshold(20.0f)
		, MaxSpinRotation(30.0f)
	{

	}

	// Basic
	//FSimpleTireConfig Tire;

	// wheel tire
	FVector Offset;
	float WheelMass;			// Mass of wheel [Kg]
	float WheelRadius;			// [cm]

	EAxleType AxleType;

	EFrictionCombineMethod FrictionCombineMethod; //#todo: use this variable
	EExternalTorqueCombineMethod ExternalTorqueCombineMethod;

	float FrictionMultiplier;
	float LateralSlipGraphMultiplier;
	float CorneringStiffness;
	float SideSlipModifier;

	float SlipThreshold;
	float SkidThreshold;

	FGraph LateralSlipGraph;
	float MaxSpinRotation;

	// #todo: simulated Damage
	//EWheelDamageStatus DamageStatus;
	//float BuckleAngle;
};


/**
* Wheel instance data changes during the simulation
*/
class VEHICLEPARTS_API FSimpleElevatorSim : public TVehicleSystem<FSimpleElevatorConfig>
{
public:

	FSimpleElevatorSim(const FSimpleElevatorConfig* SetupIn);

// Inputs

	/** Set the wheel radius - can change dynamically during simulation if desired */
	void SetWheelRadius(float NewRadius);

	/** Set the angular position in radians */
	void SetAngularPosition(float PositionIn);

	/** Set the angular position in radians/sec */
	void SetAngularVelocity(float AngularVelocityIn);

	/** set wheel rotational speed to match the specified linear forwards speed */
	void SetMatchingSpeed(float LinearMetersPerSecondIn);

	void SetTorqueCombineMethod(FSimpleElevatorConfig::EExternalTorqueCombineMethod InCombineMethod);

	/** Set the force pressing the wheel into the terrain - from suspension */
	void SetWheelLoadForce(float WheelLoadForceIn);

	void SetOnGround(bool OnGround);

	void SetMaxOmega(float InMaxOmega);

	void SetWheelIndex(uint32 InIndex);

	// Outputs

	/**
	 * Amount of friction we can expect after taking into account the amount the wheel slips
	 */
	static float GetNormalisedFrictionFromSlipAngle(float SlipIn);

	/** return the calculated available friction force */
	FVector GetForceFromFriction() const;

	/** Get the radius of the wheel [cm] */
	float GetEffectiveRadius() const;

	/** Get the angular position of the wheel [radians] */
	float GetAngularPosition() const;

	/** Get the angular velocity of the wheel [radians/sec] */
	float GetAngularVelocity() const;

	/** Get the wheel RPM [revolutions per minute] */
	float GetWheelRPM() const;

	/** Is the wheel in contact with the terrain or another object */
	bool InContact() const;


	/** Get the magnitude of the force pressing the wheel into the terrain */
	float GetWheelLoadForce() const;

	/** Get the linear ground speed of the wheel based on its current rotational speed */
	float GetWheelGroundSpeed() const;

	/** 
	 * Simulate - figure out wheel lateral and longitudinal forces based on available friction at the wheel
	 *	Wheel load force from body weight and the surface friction together determine the grip available at the wheel
	 *	DriveTorque accelerates the wheel
	 *	BrakeTorque decelerates the wheel
	 */
	void Simulate(float DeltaTime);


	void SetMassPerWheel(float VehicleMassPerWheel);

public:
	FSimpleElevatorConfig::EExternalTorqueCombineMethod ExternalTorqueCombineMethod;

	float Re;		// [cm] Effective Wheel Radius could change dynamically if get a flat?, tire shreds
	float Omega;	// [radians/sec] Wheel Rotation Angular Velocity
	float Sx;
	float Inertia;
	
	float ForceIntoSurface;			// [N]
	float AngularPosition;			// [radians]
	float MaxOmega;

	FVector ForceFromFriction;
	float MassPerWheel;

	// Wheel transform
			// Angle between wheel forwards and velocity vector
	bool bInContact;			// Is tire in contact with the ground or free in the air
	uint32 WheelIndex;			// purely for debugging purposes

	public:
	// debug for now
	float AppliedLinearDriveForce;
	float AppliedLinearBrakeForce;
	float LongitudinalAdhesiveLimit;
	float LateralAdhesiveLimit;
	float SideSlipModifier;
	float Spin;
	float AvailableGrip;
	FVector InputForces;
	bool bClipping;
};


/*
struct VEHICLEPARTS_API FAxleConfig
{
	TArray<uint16> WheelIndex;	// reference to wheels on this axle
	float RollbarScaling;
};

class VEHICLEPARTS_API FAxleSim : public TVehicleSystem<FAxleConfig>
{
public:

	FAxleSim();

	void Simulate(float DeltaTime) {}

	FAxleConfig Setup;

};
*/

} // namespace Chaos

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
