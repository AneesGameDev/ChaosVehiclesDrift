// Copyright Epic Games, Inc. All Rights Reserved.

#include "SimpleVehicle.h"
#include "ThrustSystem.h"
#include "AerofoilSystem.h"
#include "ArcadeSystem.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif


void FVehicleState::CaptureState(const FBodyInstance* TargetInstance, float GravityZ, float DeltaTime)
{
	if (TargetInstance)
	{

		VehicleWorldTransform = TargetInstance->GetUnrealWorldTransform();
		VehicleWorldVelocity = TargetInstance->GetUnrealWorldVelocity();
		VehicleWorldAngularVelocity = TargetInstance->GetUnrealWorldAngularVelocityInRadians();
		VehicleWorldCOM = TargetInstance->GetCOMPosition();
		WorldVelocityNormal = VehicleWorldVelocity.GetSafeNormal();

		VehicleUpAxis = VehicleWorldTransform.GetUnitAxis(EAxis::Z);
		VehicleForwardAxis = VehicleWorldTransform.GetUnitAxis(EAxis::X);
		VehicleRightAxis = VehicleWorldTransform.GetUnitAxis(EAxis::Y);

		VehicleLocalVelocity = VehicleWorldTransform.InverseTransformVector(VehicleWorldVelocity);
		LocalAcceleration = (VehicleLocalVelocity - LastFrameVehicleLocalVelocity) / DeltaTime;
		LocalGForce = LocalAcceleration / FMath::Abs(GravityZ);
		LastFrameVehicleLocalVelocity = VehicleLocalVelocity;

		ForwardSpeed = FVector::DotProduct(VehicleWorldVelocity, VehicleForwardAxis);
		ForwardsAcceleration = LocalAcceleration.X;
	}
}

void FVehicleState::CaptureState(const Chaos::FRigidBodyHandle_Internal* Handle, float GravityZ, float DeltaTime)
{
	if (Handle)
	{
		const FTransform WorldTM(Handle->R(), Handle->X());
		VehicleWorldTransform = WorldTM;
		VehicleWorldVelocity = Handle->V();
		VehicleWorldAngularVelocity = Handle->W();
		VehicleWorldCOM = Handle->CenterOfMass();
		WorldVelocityNormal = VehicleWorldVelocity.GetSafeNormal();

		VehicleUpAxis = VehicleWorldTransform.GetUnitAxis(EAxis::Z);
		VehicleForwardAxis = VehicleWorldTransform.GetUnitAxis(EAxis::X);
		VehicleRightAxis = VehicleWorldTransform.GetUnitAxis(EAxis::Y);

		VehicleLocalVelocity = VehicleWorldTransform.InverseTransformVector(VehicleWorldVelocity);
		LocalAcceleration = (VehicleLocalVelocity - LastFrameVehicleLocalVelocity) / DeltaTime;
		LocalGForce = LocalAcceleration / FMath::Abs(GravityZ);
		LastFrameVehicleLocalVelocity = VehicleLocalVelocity;

		ForwardSpeed = FVector::DotProduct(VehicleWorldVelocity, VehicleForwardAxis);
		ForwardsAcceleration = LocalAcceleration.X;
	}
}


namespace Chaos {
	
FVec3 IVehicleInterface::ApplyTorqueControl(const FVehicleState& VehicleState, const FControlInputs& InputData,float DeltaTime) {
	if(HasTorqueControlSetup()) {
		return GetTorqueControl().Simulate(*this,VehicleState,InputData,DeltaTime);
	}
	auto v =  FVector::Zero();
	return v;
}
FVec3 IVehicleInterface::ApplyStabilizeControl(const FVehicleState& vehicle_state, float delta_time) {
	if(HasStabilizeControlSetup()) {
		return GetStabilizeControl().Simulate(*this,vehicle_state,delta_time);
	}
	auto v =  FVector::Zero();
	return v;
}
void IVehicleInterface::ApplyAerofoils(const FControlInputs& InputData) {
	auto& af = GetAerofoils();
	for (auto& aerofoil : af) {
		aerofoil.Simulate(InputData);
	}
}

TArray<Chaos::FSimpleThrustSim>& IVehicleInterface::ApplyThrusters(const FControlInputs& ControlInputs, const FVehicleState& VehicleState, const Chaos::FRigidBodyHandle_Internal* RigidHandle,float DeltaTime) {
	auto& th = GetThrusters();
	for (auto& thruster : th) {
		thruster.Simulate(ControlInputs,VehicleState,RigidHandle,DeltaTime);
	}
	return th;
}

}
#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
