// Copyright Epic Games, Inc. All Rights Reserved.

#include "ThrustSystem.h"

#include "SimpleVehicle.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

namespace Chaos
{

	FSimpleThrustSim::FSimpleThrustSim(const FSimpleThrustConfig* SetupIn) : TVehicleSystem<FSimpleThrustConfig>(SetupIn)
		, ThrottlePosition(0.f)
		, ThrustForce(FVector::ZeroVector)
		, ThrustDirection(FVector::ZeroVector)
		, ThrusterStarted(false)
		, WorldVelocity(FVector::ZeroVector)
		, Pitch(0.f)
		, Roll(0.f)
		, Yaw(0.f)
	{

	}

	const FVector FSimpleThrustSim::GetThrustLocation() const
	{
		FVector Location = Setup().Offset;

		//if (Setup().Type == EThrustType::HelicopterRotor)
		//{
		//	Location += FVector(Pitch, -Roll, 0.f) * Setup().MaxControlAngle;
		//}

		return Location;
	}

	void FSimpleThrustSim::Simulate(const FControlInputs ControlInputs, const FVehicleState& VehicleState, const FRigidBodyHandle_Internal* RigidHandle,float DeltaTime)
	{

		SetThrottle(ControlInputs.Throttle);

		switch (Setup().Type)
		{
		case Chaos::EThrustType::HelicopterRotor:
		{
			SetPitch(ControlInputs.Pitch);
			SetRoll(ControlInputs.Roll);
		}
			break;

		case Chaos::EThrustType::Rudder:
		{
			SetYaw(-ControlInputs.Yaw - ControlInputs.Steering);
		}
			break;

		case Chaos::EThrustType::Elevator:
		{
			SetPitch(ControlInputs.Pitch);
		}
			break;

		case Chaos::EThrustType::Wing:
		{
			if (Setup().Offset.Y < 0.0f)
			{
				SetRoll(ControlInputs.Roll);
			}
			else
			{
				SetRoll(-ControlInputs.Roll);
			}
		}
			break;

		}
		
		SetWorldVelocity(VehicleState.VehicleWorldVelocity);
		ThrustDirection = Setup().Axis;

		//if (Setup().Type != EThrustType::HelicopterRotor)
		//{
		//	FRotator SteeringRotator(Pitch, Yaw, Roll);
		//	ThrustDirection = SteeringRotator.RotateVector(ThrustDirection);
		//}
		ThrustForce = ThrustDirection * (ThrottlePosition * Setup().MaxThrustForce);
	}

} // namespace Chaos

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
