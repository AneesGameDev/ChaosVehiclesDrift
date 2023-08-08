// Copyright Epic Games, Inc. All Rights Reserved.

#include "ArcadeSystem.h"
#include "SimpleVehicle.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

namespace Chaos
{
	FVec3 FTorqueControlSim::Simulate(IVehicleInterface& physicsVehicle, const FVehicleState& VehicleState,
		const FControlInputs& InputData, float DeltaTime) const {
		FVector TotalTorque = FVector::ZeroVector;
		if (Setup().Enabled)
		{
			const FTargetRotationControlConfig& TargetRotationControl = physicsVehicle.GetTargetRotationControl().Setup();
			auto ComputeTorque = [](const FVector& TargetUp, const FVector& CurrentUp, const FVector& AngVelocityWorld, float Stiffness, float Damping, float MaxAccel) -> FVector
			{
				const FQuat CurUpToTargetUp = FQuat::FindBetweenNormals(CurrentUp, TargetUp);
				const FVector Axis = CurUpToTargetUp.GetRotationAxis();
				const float Angle = CurUpToTargetUp.GetAngle();

				float Strength = (Angle * Stiffness - FVector::DotProduct(AngVelocityWorld, Axis) * Damping);
				Strength = FMath::Clamp(Strength, -MaxAccel, MaxAccel);
				const FVector Torque = Axis * Strength;
				return Torque;
			};


			FVector TargetUp = FVector(0.f, 0.f, 1.f);
			float RollMaxAngleRadians = Chaos::DegToRad(TargetRotationControl.RollMaxAngle);
			float PitchMaxAngleRadians = Chaos::DegToRad(TargetRotationControl.PitchMaxAngle);
			float Speed = FMath::Min(Chaos::CmToM(VehicleState.ForwardSpeed), 20.0f); // cap here

			float SpeeScaledRollAmount = 1.0f;
			float TargetRoll = 0.f;
			if (TargetRotationControl.bRollVsSpeedEnabled)
			{
				if (physicsVehicle.IsGrounded(0)) // HACK need IsAllowedToSteer virtual method
				{
					TargetRoll = InputData.Steering * TargetRotationControl.RollControlScaling * (Speed * Speed) * DeltaTime * 60.0f;
				}
			}
			else
			{
				TargetRoll = InputData.Steering * TargetRotationControl.RollControlScaling;
			}

			FVector Rt = VehicleState.VehicleRightAxis * FMath::Max(FMath::Min(TargetRoll, RollMaxAngleRadians), -RollMaxAngleRadians);
			FVector Pt = VehicleState.VehicleForwardAxis * FMath::Max(FMath::Min(InputData.Pitch * TargetRotationControl.PitchControlScaling, PitchMaxAngleRadians), -PitchMaxAngleRadians);

			FVector UseUp = TargetUp + Rt + Pt;
			UseUp.Normalize();

			TargetUp = UseUp;

			const FVector UpVector = VehicleState.VehicleUpAxis;
			const FVector AngVelocityWorld = VehicleState.VehicleWorldAngularVelocity;

			const FVector AirControlTorque = ComputeTorque(TargetUp, UpVector, AngVelocityWorld, TargetRotationControl.RotationStiffness, TargetRotationControl.RotationDamping, TargetRotationControl.MaxAccel);
			const FVector ForwardVector = VehicleState.VehicleForwardAxis;
			const FVector RightVector = VehicleState.VehicleRightAxis;

			const float RollAirControl = FVector::DotProduct(AirControlTorque, ForwardVector);
			const float PitchAirControl = FVector::DotProduct(AirControlTorque, RightVector);
			const float YawAirControl = FVector::DotProduct(AirControlTorque, UpVector);

			TotalTorque = RollAirControl * ForwardVector * TargetRotationControl.AutoCentreRollStrength
				+ YawAirControl * UpVector * TargetRotationControl.AutoCentreYawStrength
				+ PitchAirControl * RightVector * TargetRotationControl.AutoCentrePitchStrength;
				
			const Chaos::FTorqueControlConfig& TorqueControl = Setup();

			TotalTorque -= VehicleState.VehicleForwardAxis * InputData.Roll * TorqueControl.RollTorqueScaling;
			TotalTorque += VehicleState.VehicleRightAxis * InputData.Pitch * TorqueControl.PitchTorqueScaling;
			TotalTorque += VehicleState.VehicleUpAxis * InputData.Yaw * TorqueControl.YawTorqueScaling;
			TotalTorque += VehicleState.VehicleUpAxis * InputData.Roll * TorqueControl.YawFromRollTorqueScaling;

			// slowing rotation effect
			FVector DampingTorque = (VehicleState.VehicleWorldAngularVelocity) * TorqueControl.RotationDamping;

			GEngine->AddOnScreenDebugMessage(-1,0.1,FColor::Red,TotalTorque.ToString());
			// combined world torque
			TotalTorque -= DampingTorque;
		}

		return  TotalTorque;
	}

	FTorqueControlSim::FTorqueControlSim(const FTorqueControlConfig* SetupIn) 
		: TVehicleSystem<FTorqueControlConfig>(SetupIn)
	{

	}

	FTargetRotationControlSim::FTargetRotationControlSim(const FTargetRotationControlConfig* SetupIn) 
		: TVehicleSystem<FTargetRotationControlConfig>(SetupIn)
	{

	}

	FStabilizeControlSim::FStabilizeControlSim(const FStabilizeControlConfig* SetupIn) 
		: TVehicleSystem<FStabilizeControlConfig>(SetupIn)
	{

	}

	FVec3 FStabilizeControlSim::Simulate(IVehicleInterface& physicsVehicle,const FVehicleState& vehicle_state, float delta_time) {
		FVector CorrectionalForce = FVector::Zero();
		auto& StabilizeControl = physicsVehicle.GetStabilizeControl().Setup();
			bool MaintainAltitude = true;
			if (MaintainAltitude)
			{
				CorrectionalForce.Z = -StabilizeControl.AltitudeHoldZ * vehicle_state.VehicleWorldVelocity.Z / delta_time;
			}

		// try to cancel out velocity on X/Y plane
		// #todo: Will break helicopter setup??if (FMath::Abs(RollInput) < SMALL_NUMBER && FMath::Abs(PitchInput) < SMALL_NUMBER)
		{
			CorrectionalForce.X = -StabilizeControl.PositionHoldXY * vehicle_state.VehicleWorldVelocity.X / delta_time;
			CorrectionalForce.Y = -StabilizeControl.PositionHoldXY * vehicle_state.VehicleWorldVelocity.Y / delta_time;
		}
		return  CorrectionalForce;
	}
} // namespace Chaos

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif
