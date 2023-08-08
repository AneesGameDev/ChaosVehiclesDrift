// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once


#include "ElevatorSystem.h"
#include "Chaos/Core.h"
#include "TransmissionSystem.h"
#include "VehicleUtility.h"
#include "SimModule/SimulationModuleBase.h"
//#include "FuelSystem.h"
//#include "BallastSystem.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

/** Commonly used state - evaluated once used wherever required */
struct VEHICLEPARTS_API FVehicleState {
	FVehicleState()
	 : VehicleWorldTransform(FTransform::Identity)
	 , VehicleWorldVelocity(FVector::ZeroVector)
	 , VehicleLocalVelocity(FVector::ZeroVector)
	 , VehicleWorldAngularVelocity(FVector::ZeroVector)
	 , VehicleWorldCOM(FVector::ZeroVector)
	 , WorldVelocityNormal(FVector::ZeroVector)
	 , VehicleUpAxis(FVector(0.f,0.f,1.f))
	 , VehicleForwardAxis(FVector(1.f,0.f,0.f))
	 , VehicleRightAxis(FVector(0.f,1.f,0.f))
	 , LocalAcceleration(FVector::ZeroVector)
	 , LocalGForce(FVector::ZeroVector)
	 , LastFrameVehicleLocalVelocity(FVector::ZeroVector)
	 , ForwardSpeed(0.f)
	 , ForwardsAcceleration(0.f)
	 , NumWheelsOnGround(0)
	 , bAllWheelsOnGround(false)
	 , bVehicleInAir(true)
	 , bSleeping(false)
	 , SleepCounter(0)
	{

	}

	/** Cache some useful data at the start of the frame from GT BodyInstance */
	void CaptureState(const FBodyInstance* TargetInstance, float GravityZ, float DeltaTime);

	/** Cache some useful data at the start of the frame from Physics thread Particle Handle */
	void CaptureState(const Chaos::FRigidBodyHandle_Internal* Handle, float GravityZ, float DeltaTime);

	FTransform VehicleWorldTransform;
	FVector VehicleWorldVelocity;
	FVector VehicleLocalVelocity;
	FVector VehicleWorldAngularVelocity;
	FVector VehicleWorldCOM;
	FVector WorldVelocityNormal;

	FVector VehicleUpAxis;
	FVector VehicleForwardAxis;
	FVector VehicleRightAxis;
	FVector LocalAcceleration;
	FVector LocalGForce;
	FVector LastFrameVehicleLocalVelocity;

	float ForwardSpeed;
	float ForwardsAcceleration;

	int NumWheelsOnGround;
	bool bAllWheelsOnGround;
	bool bVehicleInAir;
	bool bSleeping;
	int SleepCounter;
};

namespace Chaos {
 class FSimpleAerodynamicsSim;
 class FSimpleThrustSim;
 class FAerofoil;
 class FStabilizeControlSim;
 class FTorqueControlSim;
 class FTargetRotationControlSim;

	class VEHICLEPARTS_API IVehicleInterface {
    public:
     virtual ~IVehicleInterface() = default;
     virtual const FTargetRotationControlSim& GetTargetRotationControl() = 0;
     virtual const bool IsGrounded(int MoverIndex = -1) = 0;

#pragma region TORQUE_CONTROL
     FVec3 ApplyTorqueControl(const FVehicleState& VehicleState, const FControlInputs& InputData,float DeltaTime);
     virtual bool HasTorqueControlSetup() = 0;
     virtual FTorqueControlSim& GetTorqueControl() = 0;
#pragma endregion
  
#pragma region STABILIZE_CONTROL
     virtual bool HasStabilizeControlSetup() = 0;
     virtual FStabilizeControlSim& GetStabilizeControl() = 0;
     FVec3 ApplyStabilizeControl(const FVehicleState& vehicle_state, float delta_time);
#pragma endregion

#pragma region AEROFOILS
     virtual TArray<FAerofoil>& GetAerofoils() = 0;
     void ApplyAerofoils(const FControlInputs& InputData);
#pragma endregion
		
#pragma region THRUSTERS
     virtual TArray<FSimpleThrustSim>& GetThrusters() = 0;
     TArray<FSimpleThrustSim>& ApplyThrusters(const FControlInputs& ControlInputs, const FVehicleState& VehicleState, const FRigidBodyHandle_Internal* RigidHandle, float DeltaTime);
#pragma endregion
		
#pragma region TRANSMISSION
     virtual bool HasTransmission() const = 0;
     virtual FSimpleTransmissionSim& GetTransmission() = 0;
#pragma endregion
		
#pragma region AERODYNAMICS
     virtual FSimpleAerodynamicsSim& GetAerodynamics() = 0;
#pragma endregion 

     //	virtual void Simulate(float DeltaTime) = 0;
    };
}

/**
 * This class is currently just a container for the simulation components used by a wheeled vehicle
 * Keeping all the physics systems together and accessible through the one vehicle class
 */

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif


