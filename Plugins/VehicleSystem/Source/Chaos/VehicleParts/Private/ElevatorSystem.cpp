// Copyright Epic Games, Inc. All Rights Reserved.

#include "ElevatorSystem.h"

#include "WheelSystem.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

namespace Chaos
{
	FSimpleElevatorSim::FSimpleElevatorSim(const FSimpleElevatorConfig* SetupIn) : TVehicleSystem<FSimpleElevatorConfig>(SetupIn)
		, ExternalTorqueCombineMethod(SetupIn->ExternalTorqueCombineMethod)
		, Re(SetupIn->WheelRadius)
		, Omega(0.f)
		, Sx(0.f)
		, Inertia(0.5f * SetupIn->WheelMass * SetupIn->WheelRadius * SetupIn->WheelRadius)
		, ForceIntoSurface(0.f)
		, AngularPosition(0.f)
		, ForceFromFriction(FVector::ZeroVector)
		, MassPerWheel(250.f)
		, bInContact(false)
		, WheelIndex(0)
		, Spin(0.f)
		, AvailableGrip(0.f)
		, InputForces(FVector::ZeroVector)
		, bClipping(false)
	{

	}

	void FSimpleElevatorSim::SetWheelRadius(float NewRadius) {
		Re = NewRadius;
	}

	void FSimpleElevatorSim::SetAngularPosition(float PositionIn) {
		AngularPosition = PositionIn;
	}

	void FSimpleElevatorSim::SetAngularVelocity(float AngularVelocityIn) {
		Omega = AngularVelocityIn;
	}

	void FSimpleElevatorSim::SetMatchingSpeed(float LinearMetersPerSecondIn) {
		Omega = LinearMetersPerSecondIn / Re;
	}

	void FSimpleElevatorSim::
	SetTorqueCombineMethod(FSimpleElevatorConfig::EExternalTorqueCombineMethod InCombineMethod) {
		ExternalTorqueCombineMethod = InCombineMethod;
	}

	void FSimpleElevatorSim::SetWheelLoadForce(float WheelLoadForceIn) {
		ForceIntoSurface = WheelLoadForceIn;
		
		if (ForceIntoSurface > SMALL_NUMBER)
		{
			bInContact = true;
		}
		else
		{
			bInContact = false;
		}
	}

	void FSimpleElevatorSim::SetOnGround(bool OnGround) {
		bInContact = OnGround;
	}

	void FSimpleElevatorSim::SetMaxOmega(float InMaxOmega) {
		MaxOmega = InMaxOmega;
	}

	void FSimpleElevatorSim::SetWheelIndex(uint32 InIndex) {
		WheelIndex = InIndex;
	}

	float FSimpleElevatorSim::GetNormalisedFrictionFromSlipAngle(float SlipIn) {
		FVehicleUtility::ClampNormalRange(SlipIn);

		// typical slip angle graph; normalized scales
		// Friction between 0 and 1 for values of slip between 0 and 1
		float FunctionResult = 1.125f * (1.0f - exp(-20.0f * SlipIn)) - 0.25f * SlipIn;
		return FMath::Max(0.0f, FMath::Min(1.0f, FunctionResult));
	}

	FVector FSimpleElevatorSim::GetForceFromFriction() const {
		return ForceFromFriction;
	}

	float FSimpleElevatorSim::GetEffectiveRadius() const {
		return Re;
	}

	float FSimpleElevatorSim::GetAngularPosition() const {
		return AngularPosition;
	}

	float FSimpleElevatorSim::GetAngularVelocity() const {
		return Omega;
	}

	float FSimpleElevatorSim::GetWheelRPM() const {
		return OmegaToRPM(Omega);
	}

	bool FSimpleElevatorSim::InContact() const {
		return bInContact;
	}

	float FSimpleElevatorSim::GetWheelLoadForce() const {
		return ForceIntoSurface;
	}

	float FSimpleElevatorSim::GetWheelGroundSpeed() const {
		return Omega * Re;
	}

	void FSimpleElevatorSim::Simulate(float DeltaTime) {
	}

	void FSimpleElevatorSim::SetMassPerWheel(float VehicleMassPerWheel) {
		MassPerWheel = VehicleMassPerWheel;
	}


	/*
	FAxleSim::FAxleSim() : TVehicleSystem<FAxleConfig>(&Setup)
	{
	}
	*/



} // namespace Chaos

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
