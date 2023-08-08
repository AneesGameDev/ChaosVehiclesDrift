// Copyright Epic Games, Inc. All Rights Reserved.

#include "ChaosWheeledVehicleMovementComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "VehicleAnimationInstance.h"
#include "ChaosVehicleManager.h"
#include "ChaosVehicleWheel.h"
#include "SuspensionUtility.h"
#include "SteeringUtility.h"
#include "TransmissionUtility.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PBDSuspensionConstraintData.h"
#include "Chaos/DebugDrawQueue.h"
#include "UObject/UE5MainStreamObjectVersion.h"

#include "PhysicsProxy/SuspensionConstraintProxy.h"
#include "PBDRigidsSolver.h"

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
#include "CanvasItem.h"
#include "Engine/Canvas.h"
#endif
#include "WheeledSnapshotData.h"
#include "..\Public\ChaosWheeledVehicleManagerAsyncCallback.h"
#include "GameFramework/Controller.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

DECLARE_STATS_GROUP(TEXT("ChaosVehicle"), STATGROUP_ChaosVehicle, STATGROUP_Advanced);

DECLARE_CYCLE_STAT(TEXT("Vehicle:SuspensionRaycasts"), STAT_ChaosVehicle_SuspensionRaycasts, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:SuspensionOverlapTest"), STAT_ChaosVehicle_SuspensionOverlapTest, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:SuspensionTraces"), STAT_ChaosVehicle_SuspensionTraces, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:TickVehicle"), STAT_ChaosVehicle_TickVehicle, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:UpdateSimulation"), STAT_ChaosVehicle_UpdateSimulation, STATGROUP_ChaosVehicle);


FWheeledVehicleDebugParams GWheeledVehicleDebugParams;

FAutoConsoleVariableRef CVarWheeledVehiclesShowWheelCollisionNormal(TEXT("p.Vehicle.ShowWheelCollisionNormal"), GWheeledVehicleDebugParams.ShowWheelCollisionNormal, TEXT("Enable/Disable Wheel Collision Normal Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesShowSuspensionRaycasts(TEXT("p.Vehicle.ShowSuspensionRaycasts"), GWheeledVehicleDebugParams.ShowSuspensionRaycasts, TEXT("Enable/Disable Suspension Raycast Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesShowSuspensionLimits(TEXT("p.Vehicle.ShowSuspensionLimits"), GWheeledVehicleDebugParams.ShowSuspensionLimits, TEXT("Enable/Disable Suspension Limits Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesShowWheelForces(TEXT("p.Vehicle.ShowWheelForces"), GWheeledVehicleDebugParams.ShowWheelForces, TEXT("Enable/Disable Wheel Forces Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesShowSuspensionForces(TEXT("p.Vehicle.ShowSuspensionForces"), GWheeledVehicleDebugParams.ShowSuspensionForces, TEXT("Enable/Disable Suspension Forces Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesShowBatchQueryExtents(TEXT("p.Vehicle.ShowBatchQueryExtents"), GWheeledVehicleDebugParams.ShowBatchQueryExtents, TEXT("Enable/Disable Suspension Forces Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesShowRaycastComponent(TEXT("p.Vehicle.ShowRaycastComponent"), GWheeledVehicleDebugParams.ShowRaycastComponent, TEXT("Enable/Disable Raycast Component Hit Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesShowRaycastMaterial(TEXT("p.Vehicle.ShowRaycastMaterial"), GWheeledVehicleDebugParams.ShowRaycastMaterial, TEXT("Enable/Disable Raycast Material Hit Visualisation."));
FAutoConsoleVariableRef CVarWheeledVehiclesTraceTypeOverride(TEXT("p.Vehicle.TraceTypeOverride"), GWheeledVehicleDebugParams.TraceTypeOverride, TEXT("Override ray trace type, 1=Simple, 2=Complex."));

FAutoConsoleVariableRef CVarWheeledVehiclesDisableSuspensionForces(TEXT("p.Vehicle.DisableSuspensionForces"), GWheeledVehicleDebugParams.DisableSuspensionForces, TEXT("Enable/Disable Suspension Forces."));
FAutoConsoleVariableRef CVarWheeledVehiclesDisableFrictionForces(TEXT("p.Vehicle.DisableFrictionForces"), GWheeledVehicleDebugParams.DisableFrictionForces, TEXT("Enable/Disable Wheel Friction Forces."));
FAutoConsoleVariableRef CVarWheeledVehiclesDisableRollbarForces(TEXT("p.Vehicle.DisableRollbarForces"), GWheeledVehicleDebugParams.DisableRollbarForces, TEXT("Enable/Disable Rollbar Forces."));
FAutoConsoleVariableRef CVarWheeledVehiclesDisableConstraintSuspension(TEXT("p.Vehicle.DisableConstraintSuspension"), GWheeledVehicleDebugParams.DisableConstraintSuspension, TEXT("Enable/Disable Constraint based suspension, swaps to basic force based suspension without hardstops instead."));

FAutoConsoleVariableRef CVarWheeledVehiclesThrottleOverride(TEXT("p.Vehicle.ThrottleOverride"), GWheeledVehicleDebugParams.ThrottleOverride, TEXT("Hard code throttle input on."));
FAutoConsoleVariableRef CVarWheeledVehiclesSteeringOverride(TEXT("p.Vehicle.SteeringOverride"), GWheeledVehicleDebugParams.SteeringOverride, TEXT("Hard code steering input on."));

FAutoConsoleVariableRef CVarWheeledVehiclesResetMeasurements(TEXT("p.Vehicle.ResetMeasurements"), GWheeledVehicleDebugParams.ResetPerformanceMeasurements, TEXT("Reset Vehicle Performance Measurements."));

FAutoConsoleVariableRef CVarWheeledVehiclesOverlapTestExpansionXY(TEXT("p.Vehicle.OverlapTestExpansionXY"), GWheeledVehicleDebugParams.OverlapTestExpansionXY, TEXT("Raycast Overlap Test Expansion of Bounding Box in X/Y axes."));
FAutoConsoleVariableRef CVarWheeledVehiclesOverlapTestExpansionXZ(TEXT("p.Vehicle.OverlapTestExpansionZ"), GWheeledVehicleDebugParams.OverlapTestExpansionZ, TEXT("Raycast Overlap Test Expansion of Bounding Box in Z axis"));

//FAutoConsoleVariableRef CVarWheeledVehiclesDisableSuspensionConstraints(TEXT("p.Vehicle.DisableSuspensionConstraint"), GWheeledVehicleDebugParams.DisableSuspensionConstraint, TEXT("Enable/Disable Suspension Constraints."));

FString FWheelStatus::ToString() const {
	return FString::Printf(TEXT("bInContact:%s ContactPoint:%s PhysMaterial:%s NormSuspensionLength:%f SpringForce:%f SlipAngle:%f bIsSlipping:%s SlipMagnitude:%f bIsSkidding:%s SkidMagnitude:%f SkidNormal:%s"), bInContact == true ? TEXT("True") : TEXT("False"), *ContactPoint.ToString(), PhysMaterial.IsValid() ? *PhysMaterial->GetName() : TEXT("None"), NormalizedSuspensionLength, SpringForce, SlipAngle, bIsSlipping == true ? TEXT("True") : TEXT("False"), SlipMagnitude, bIsSkidding == true ? TEXT("True") : TEXT("False"), SkidMagnitude, *SkidNormal.ToString());
}

void FWheelState::CaptureState(int WheelIdx, const FVector& WheelOffset, const FBodyInstance* TargetInstance) {
	check(TargetInstance);
	const FTransform WorldTransform = TargetInstance->GetUnrealWorldTransform();
	WheelLocalLocation[WheelIdx] = WheelOffset;
	WheelWorldLocation[WheelIdx] = WorldTransform.TransformPosition(WheelOffset);
	WorldWheelVelocity[WheelIdx] = TargetInstance->GetUnrealWorldVelocityAtPoint(WheelWorldLocation[WheelIdx]);
	LocalWheelVelocity[WheelIdx] = WorldTransform.InverseTransformVector(WorldWheelVelocity[WheelIdx]);
}

void FWheelState::CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* Handle) {
	check(Handle);
	const FTransform WorldTransform(Handle->R(), Handle->X());
	WheelLocalLocation[WheelIdx] = WheelOffset;
	WheelWorldLocation[WheelIdx] = WorldTransform.TransformPosition(WheelOffset);
	WorldWheelVelocity[WheelIdx] = GetVelocityAtPoint(Handle, WheelWorldLocation[WheelIdx]);
	LocalWheelVelocity[WheelIdx] = WorldTransform.InverseTransformVector(WorldWheelVelocity[WheelIdx]);
}

void FWheelState::CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* VehicleHandle, const FVector& ContactPoint, const Chaos::FRigidBodyHandle_Internal* SurfaceHandle) {
	check(VehicleHandle);

	FVector SurfaceVelocity = FVector::ZeroVector;
	if (SurfaceHandle) {
		SurfaceVelocity = GetVelocityAtPoint(SurfaceHandle, ContactPoint);
	}

	const FTransform WorldTransform(VehicleHandle->R(), VehicleHandle->X());
	WheelLocalLocation[WheelIdx] = WheelOffset;
	WheelWorldLocation[WheelIdx] = WorldTransform.TransformPosition(WheelOffset);
	WorldWheelVelocity[WheelIdx] = GetVelocityAtPoint(VehicleHandle, WheelWorldLocation[WheelIdx]) - SurfaceVelocity;
	LocalWheelVelocity[WheelIdx] = WorldTransform.InverseTransformVector(WorldWheelVelocity[WheelIdx]);
}

FVector FWheelState::GetVelocityAtPoint(const Chaos::FRigidBodyHandle_Internal* Rigid, const FVector& InPoint) {
	if (Rigid) {
		const Chaos::FVec3 COM = Rigid ? Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(Rigid) : (Chaos::FVec3)Chaos::FParticleUtilitiesGT::GetActorWorldTransform(Rigid).GetTranslation();
		const Chaos::FVec3 Diff = InPoint - COM;
		return Rigid->V() - Chaos::FVec3::CrossProduct(Diff, Rigid->W());
	}
	else {
		return FVector::ZeroVector;
	}
}

/**
 * UChaosWheeledVehicleSimulation
 */
bool UChaosWheeledVehicleSimulation::CanSimulate() const {
	if (UChaosVehicleSimulation::CanSimulate() == false) {
		return false;
	}

	return (PVehicle && PVehicle->IsValid() && PVehicle->Engine.Num() == PVehicle->Transmission.Num() && PVehicle->Wheels.Num() == PVehicle->Suspension.Num());
}

void UChaosWheeledVehicleSimulation::TickVehicle(UWorld* WorldIn, float DeltaTime, const FWheeledVehicleAsyncInput& InputData, FWheeledVehicleAsyncOutput& OutputData, Chaos::FRigidBodyHandle_Internal* Handle) {
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_TickVehicle);


	World = WorldIn;
	RigidHandle = Handle;

	// movement updates and replication
	if (World && RigidHandle) {
		if (!VehicleState.bSleeping) {
			if (CanSimulate() && Handle) {
				UpdateSimulation(DeltaTime, InputData, Handle);
				FillOutputState(OutputData);
			}
		}
	}


#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	DrawDebug3D();
#endif
}

#define GETPRESET(X) (isDrifting?DriftSetup:GripSetup).X

void UChaosWheeledVehicleSimulation::UpdateDriftingState(const FControlInputs& control_inputs) {
	bool WasDrifting = isDrifting;
	if (control_inputs.HandbrakeInput) {
		if (!isPressed) {
			isDrifting = !isDrifting;
		}
		isPressed = true;
	}
	isPressed = control_inputs.HandbrakeInput;
	if (WasDrifting ^ isDrifting) {
		GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Blue, FString(isDrifting ? "Drift" : "Grip"));
		PVehicle->GetAerodynamics().SetDownforceCoefficient(isDrifting ? DriftSetup.DownforceCoefficient : GripSetup.DownforceCoefficient);
		PVehicle->GetDifferential().FrontRearSplit = isDrifting ? DriftSetup.FrontRearSplit : GripSetup.FrontRearSplit;

		for (int i = 0; i < GETPRESET(WheelsMaxSteerAngle).Num(); i++) {
			PVehicle->Wheels[i].MaxSteeringAngle = GETPRESET(WheelsMaxSteerAngle)[i];
		}
		for (int i = 0; i < GETPRESET(WheelLoadRatio).Num(); i++) {
			PVehicle->Suspension[i].AccessSetup().WheelLoadRatio = GETPRESET(WheelLoadRatio)[i];
		}
		for (int i = 0; i < GETPRESET(WheelSideSlipModifier).Num(); i++) {
			PVehicle->Wheels[i].SideSlipModifier = GETPRESET(WheelSideSlipModifier)[i];
		}
		for (int i = 0; i < GETPRESET(WheelCorneringStiffness).Num(); i++) {
			PVehicle->Wheels[i].CorneringStiffness = GETPRESET(WheelCorneringStiffness)[i] * 10000.f;
		}
		for (int i = 0; i < GETPRESET(WheelFrictionMultiplier).Num(); i++) {
			PVehicle->Wheels[i].FrictionMultiplier = GETPRESET(WheelFrictionMultiplier)[i];
		}
		for (int i = 0; i < GETPRESET(WheelLateralSlipGraph).Num(); i++) {
			auto& VehicleWheel = PVehicle->Wheels[i];

			VehicleWheel.AccessSetup().LateralSlipGraph.Empty();

			auto LateralSlipGraph = GETPRESET(WheelLateralSlipGraph)[i];
			float NumSamples = 20;
			float MinTime = 0.f, MaxTime = 0.f;
			LateralSlipGraph.GetRichCurveConst()->GetTimeRange(MinTime, MaxTime);

			if (MaxTime > 0.0f) {
				for (float X = 0; X <= MaxTime; X += (MaxTime / NumSamples)) {
					float MinVal = 0.f, MaxVal = 0.f;
					LateralSlipGraph.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
					float Y = LateralSlipGraph.GetRichCurveConst()->Eval(X) * 10000.0f;
					VehicleWheel.AccessSetup().LateralSlipGraph.Add(Chaos::FVec2(X, Y));
				}
			}
		}
		for (int i = 0; i < GETPRESET(bWheelTractionControll).Num(); i++) {
			PVehicle->Wheels[i].TractionControlEnabled = GETPRESET(bWheelTractionControll)[i];
		}
	}
}

void UChaosWheeledVehicleSimulation::UpdateSimulation(float DeltaTime, const FWheeledVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) {
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_UpdateSimulation);

	VehicleState.CaptureState(Handle, InputData.GravityZ, DeltaTime);

	UpdateDriftingState(InputData.ControlInputs);

	ApplyAerodynamics(DeltaTime);
	ApplyAerofoilForces(DeltaTime);
	ApplyThrustForces(DeltaTime, InputData.ControlInputs);
	ApplyTorqueControl(DeltaTime, InputData.ControlInputs);

	{
		// sanity check that everything is setup ok
		ensure(PVehicle->Wheels.Num() == PVehicle->Suspension.Num());
		ensure(WheelState.LocalWheelVelocity.Num() == PVehicle->Wheels.Num());
		ensure(WheelState.WheelWorldLocation.Num() == PVehicle->Wheels.Num());
		ensure(WheelState.WorldWheelVelocity.Num() == PVehicle->Wheels.Num());

		///////////////////////////////////////////////////////////////////////
		// Cache useful state so we are not re-calculating the same data
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++) {
			bool bCaptured = false;

			// #TODO: This is not threadsafe - need to rethink how to get the rigidbody that is hit by the raycast
			//const FHitResult& HitResult = Wheels[WheelIdx]->HitResult;
			//if (HitResult.Component.IsValid() && HitResult.Component->GetBodyInstance())
			//{
			//	if (const FPhysicsActorHandle& SurfaceHandle = HitResult.Component->GetBodyInstance()->GetPhysicsActorHandle())
			//	{
			//		// we are being called from the physics thread
			//		if (Chaos::FRigidBodyHandle_Internal* SurfaceBody = SurfaceHandle->GetPhysicsThreadAPI())
			//		{
			//			if (SurfaceBody->CanTreatAsKinematic())
			//			{
			//				FVector Point = HitResult.ImpactPoint;
			//				WheelState.CaptureState(WheelIdx, PVehicle->Suspension[WheelIdx].GetLocalRestingPosition(), Handle, Point, SurfaceBody);
			//				bCaptured = true;
			//			}
			//		}
			//	}
			//}

			if (!bCaptured) {
				WheelState.CaptureState(WheelIdx, PVehicle->Suspension[WheelIdx].GetLocalRestingPosition(), Handle);
			}
		}

		///////////////////////////////////////////////////////////////////////
		// Suspension Raycast

		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++) {
			auto& PSuspension = PVehicle->Suspension[WheelIdx];
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			PSuspension.UpdateWorldRaycastLocation(VehicleState.VehicleWorldTransform, PWheel.GetEffectiveRadius(), WheelState.Trace[WheelIdx]);
		}

		if (!GWheeledVehicleDebugParams.DisableSuspensionForces && PVehicle->bSuspensionEnabled) {
			PerformSuspensionTraces(WheelState.Trace, InputData.TraceParams, InputData.TraceCollisionResponse, InputData.WheelTraceParams);
		}

		//////////////////////////////////////////////////////////////////////////
		// Wheel and Vehicle in air state

		VehicleState.bVehicleInAir = true;
		VehicleState.NumWheelsOnGround = 0;
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++) {
			// tell systems who care that wheel is touching the ground
			PVehicle->Wheels[WheelIdx].SetOnGround(WheelState.TraceResult[WheelIdx].bBlockingHit);

			// only requires one wheel to be on the ground for the vehicle to be NOT in the air
			if (PVehicle->Wheels[WheelIdx].InContact()) {
				VehicleState.bVehicleInAir = false;
				VehicleState.NumWheelsOnGround++;
			}
		}
		VehicleState.bAllWheelsOnGround = (VehicleState.NumWheelsOnGround == PVehicle->Wheels.Num());

		///////////////////////////////////////////////////////////////////////
		// Input
		ApplyInput(InputData.ControlInputs, DeltaTime);

		///////////////////////////////////////////////////////////////////////
		// Engine/Transmission
		if (!GWheeledVehicleDebugParams.DisableSuspensionForces && PVehicle->bMechanicalSimEnabled) {
			ProcessMechanicalSimulation(DeltaTime);
		}

		///////////////////////////////////////////////////////////////////////
		// Suspension

		if (!GWheeledVehicleDebugParams.DisableSuspensionForces && PVehicle->bSuspensionEnabled) {
			ApplySuspensionForces(DeltaTime);
		}

		///////////////////////////////////////////////////////////////////////
		// Steering

		ProcessSteering(InputData.ControlInputs);

		///////////////////////////////////////////////////////////////////////
		// Wheel Friction

		if (!GWheeledVehicleDebugParams.DisableFrictionForces && PVehicle->bWheelFrictionEnabled) {
			ApplyWheelFrictionForces(DeltaTime);
		}

#if 0
		if (PerformanceMeasure.IsEnabled())
		{
			PerformanceMeasure.Update(DeltaTime, VehicleState.VehicleWorldTransform.GetLocation(), VehicleState.ForwardSpeed);
		}
#endif
	}
}

bool UChaosWheeledVehicleSimulation::ContainsTraces(const FBox& Box, const TArray<Chaos::FSuspensionTrace>& SuspensionTrace) {
	const Chaos::FAABB3 Aabb(Box.Min, Box.Max);

	for (int WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); WheelIdx++) {
		if (!Aabb.Contains(SuspensionTrace[WheelIdx].Start) || !Aabb.Contains(SuspensionTrace[WheelIdx].End)) {
			return false;
		}
	}

	return true;
}

void UChaosWheeledVehicleSimulation::PerformSuspensionTraces(const TArray<Chaos::FSuspensionTrace>& SuspensionTrace, FCollisionQueryParams& TraceParams, FCollisionResponseContainer& CollisionResponse, TArray<FWheelTraceParams>& WheelTraceParams) {
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionRaycasts);

	ECollisionChannel SpringCollisionChannel = ECollisionChannel::ECC_WorldDynamic;
	FCollisionResponseParams ResponseParams;
	ResponseParams.CollisionResponse = CollisionResponse;

	// batching is about 0.5ms (25%) faster when there's 100 vehicles on a flat terrain
	if (GVehicleDebugParams.BatchQueries) {
		if (!GVehicleDebugParams.CacheTraceOverlap || !ContainsTraces(QueryBox, SuspensionTrace)) {
			SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionOverlapTest);

			bOverlapHit = false;
			OverlapResults.Empty();
			QueryBox.Init();

			//FBox QueryBox;
			for (int WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); WheelIdx++) {
				const FVector& TraceStart = SuspensionTrace[WheelIdx].Start;
				const FVector& TraceEnd = SuspensionTrace[WheelIdx].End;

				if (WheelIdx == 0) {
					QueryBox = FBox(TraceStart, TraceEnd);
				}
				else {
					QueryBox.Min = QueryBox.Min.ComponentMin(TraceStart);
					QueryBox.Min = QueryBox.Min.ComponentMin(TraceEnd);
					QueryBox.Max = QueryBox.Max.ComponentMax(TraceStart);
					QueryBox.Max = QueryBox.Max.ComponentMax(TraceEnd);
				}
			}
			QueryBox = QueryBox.ExpandBy(FVector(GWheeledVehicleDebugParams.OverlapTestExpansionXY, GWheeledVehicleDebugParams.OverlapTestExpansionXY, GWheeledVehicleDebugParams.OverlapTestExpansionZ));
			FCollisionShape CollisionBox;
			CollisionBox.SetBox((FVector3f)QueryBox.GetExtent());

			bOverlapHit = World->OverlapMultiByChannel(OverlapResults, QueryBox.GetCenter(), FQuat::Identity, SpringCollisionChannel, CollisionBox, TraceParams, ResponseParams);
		}

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
		if (GWheeledVehicleDebugParams.ShowBatchQueryExtents) {
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugBox(QueryBox.GetCenter(), QueryBox.GetExtent(), FQuat::Identity, FColor::Yellow, false, -1.0f, 0, 2.0f);

			// draw all corresponding results bounding boxes
			for (FOverlapResult OverlapResult : OverlapResults) {
				if (OverlapResult.bBlockingHit) {
					const FBoxSphereBounds Bounds = OverlapResult.Component->CalcBounds(OverlapResult.Component->GetComponentTransform());
					Chaos::FDebugDrawQueue::GetInstance().DrawDebugBox(Bounds.GetBox().GetCenter(), Bounds.GetBox().GetExtent(), FQuat::Identity, FColor::Purple, false, -1.0f, 0, 2.0f);
				}
			}
		}
#endif

		SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionTraces);
		for (int32 WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); ++WheelIdx) {
			FHitResult& HitResult = WheelState.TraceResult[WheelIdx];
			HitResult = FHitResult();

			if (bOverlapHit) {
				const FVector& TraceStart = SuspensionTrace[WheelIdx].Start;
				const FVector& TraceEnd = SuspensionTrace[WheelIdx].End;
				TraceParams.bTraceComplex = (WheelTraceParams[WheelIdx].SweepType == ESweepType::ComplexSweep);

				if (GWheeledVehicleDebugParams.TraceTypeOverride > 0) {
					TraceParams.bTraceComplex = GWheeledVehicleDebugParams.TraceTypeOverride == 2;
				}

				FVector TraceVector(TraceStart - TraceEnd); // reversed
				FVector TraceNormal = TraceVector.GetSafeNormal();

				// Test each overlapped object for a hit result
				for (FOverlapResult OverlapResult : OverlapResults) {
					if (!OverlapResult.bBlockingHit) continue;

					FHitResult ComponentHit;

					switch (WheelTraceParams[WheelIdx].SweepShape) {
					case ESweepShape::Spherecast: {
						float WheelRadius = PVehicle->Wheels[WheelIdx].GetEffectiveRadius();
						FVector VehicleUpAxis = TraceNormal;

						FVector Start = TraceStart + VehicleUpAxis * WheelRadius;
						FVector End = TraceEnd;

						if (OverlapResult.Component->SweepComponent(ComponentHit, Start, End, FQuat::Identity, FCollisionShape::MakeSphere(WheelRadius), TraceParams.bTraceComplex)) {
							if (ComponentHit.Time < HitResult.Time) {
								HitResult = ComponentHit;
								HitResult.bBlockingHit = OverlapResult.bBlockingHit;
							}
						}
					}
					break;

					case ESweepShape::Raycast: default: {
						if (OverlapResult.Component->LineTraceComponent(ComponentHit, TraceStart, TraceEnd, TraceParams)) {
							if (ComponentHit.Time < HitResult.Time) {
								HitResult = ComponentHit;
								HitResult.bBlockingHit = OverlapResult.bBlockingHit;
							}
						}
					}
						break;
					}
				}
			}
		}
	}
	else {
		SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionTraces);
		for (int WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); WheelIdx++) {
			FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

			FVector TraceStart = SuspensionTrace[WheelIdx].Start;
			FVector TraceEnd = SuspensionTrace[WheelIdx].End;
			TraceParams.bTraceComplex = (WheelTraceParams[WheelIdx].SweepType == ESweepType::ComplexSweep);

			if (GWheeledVehicleDebugParams.TraceTypeOverride > 0) {
				TraceParams.bTraceComplex = GWheeledVehicleDebugParams.TraceTypeOverride == 2;
			}

			FVector TraceVector(TraceStart - TraceEnd); // reversed
			FVector TraceNormal = TraceVector.GetSafeNormal();

			switch (WheelTraceParams[WheelIdx].SweepShape) {
			case ESweepShape::Spherecast: {
				float WheelRadius = PVehicle->Wheels[WheelIdx].GetEffectiveRadius();
				FVector VehicleUpAxis = TraceNormal;

				World->SweepSingleByChannel(HitResult, TraceStart + VehicleUpAxis * WheelRadius, TraceEnd, FQuat::Identity, SpringCollisionChannel, FCollisionShape::MakeSphere(WheelRadius), TraceParams, ResponseParams);
			}
			break;


			case ESweepShape::Raycast: default: {
				World->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, SpringCollisionChannel, TraceParams, ResponseParams);
			}
				break;
			}
		}
	}
}

void UChaosWheeledVehicleSimulation::ApplyWheelFrictionForces(float DeltaTime) {
	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++) {
		auto& PWheel = PVehicle->Wheels[WheelIdx]; // Physics Wheel
		FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

		if (PWheel.InContact()) {
			if (HitResult.PhysMaterial.IsValid()) {
				PWheel.SetSurfaceFriction(HitResult.PhysMaterial->Friction);
			}

			// take into account steering angle
			float SteerAngleDegrees = PWheel.SteeringAngle;
			FRotator SteeringRotator(0.f, SteerAngleDegrees, 0.f);
			FVector SteerLocalWheelVelocity = SteeringRotator.UnrotateVector(WheelState.LocalWheelVelocity[WheelIdx]);

			PWheel.SetVehicleGroundSpeed(SteerLocalWheelVelocity);
			PWheel.Simulate(DeltaTime);

			float RotationAngle = FMath::RadiansToDegrees(PWheel.GetAngularPosition());
			FVector FrictionForceLocal = PWheel.GetForceFromFriction();
			FrictionForceLocal = SteeringRotator.RotateVector(FrictionForceLocal);

			FVector GroundZVector = HitResult.Normal;
			FVector GroundXVector = FVector::CrossProduct(VehicleState.VehicleRightAxis, GroundZVector);
			FVector GroundYVector = FVector::CrossProduct(GroundZVector, GroundXVector);

			FMatrix Mat = FMatrix(GroundXVector, GroundYVector, GroundZVector, VehicleState.VehicleWorldTransform.GetLocation());
			FVector FrictionForceVector = Mat.TransformVector(FrictionForceLocal);

			check(PWheel.InContact());
			if (PVehicle->bLegacyWheelFrictionPosition) {
				AddForceAtPosition(FrictionForceVector, WheelState.WheelWorldLocation[WheelIdx]);
			}
			else {
				AddForceAtPosition(FrictionForceVector, HitResult.ImpactPoint);
			}

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
			if (GWheeledVehicleDebugParams.ShowWheelForces) {
				// show longitudinal drive force
				if (PWheel.AvailableGrip > 0.0f) {
					float Radius = 50.0f;
					float Scaling = 50.0f / PWheel.AvailableGrip;

					FVector Center = WheelState.WheelWorldLocation[WheelIdx];
					FVector Offset(0.0f, WheelState.WheelLocalLocation[WheelIdx].Y, 10.f);
					Offset = Mat.TransformVector(Offset);

					Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(Center, Center + GroundZVector * 100.f, FColor::Orange, false, -1.0f, 0, 2);

					Center += Offset;
					FVector InputForceVectorWorld = Mat.TransformVector(PWheel.InputForces);
					Chaos::FDebugDrawQueue::GetInstance().DrawDebugCircle(Center, Radius, 60, FColor::White, false, -1.0f, 0, 3, FVector(1, 0, 0), FVector(0, 1, 0), false);
					Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(Center, Center + InputForceVectorWorld * Scaling, (PWheel.bClipping ? FColor::Red : FColor::Green), false, -1.0f, 0, PWheel.bClipping ? 2 : 4);
					Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(Center, Center + FrictionForceVector * Scaling, FColor::Yellow, false, -1.0f, 1, PWheel.bClipping ? 4 : 2);
				}
			}
#endif
		}
		else {
			PWheel.SetVehicleGroundSpeed(FVector::ZeroVector);
			PWheel.SetWheelLoadForce(0.f);
			PWheel.Simulate(DeltaTime);
		}
	}
}

void UChaosWheeledVehicleSimulation::ApplySuspensionForces(float DeltaTime) {
	TArray<float> SusForces;
	SusForces.Init(0.f, PVehicle->Suspension.Num());

	for (int WheelIdx = 0; WheelIdx < SusForces.Num(); WheelIdx++) {
		FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

		float NewDesiredLength = 1.0f; // suspension max length
		float ForceMagnitude2 = 0.f;
		auto& PWheel = PVehicle->Wheels[WheelIdx];
		auto& PSuspension = PVehicle->Suspension[WheelIdx];
		float SuspensionMovePosition = -PSuspension.Setup().MaxLength;

		if (!GWheeledVehicleDebugParams.DisableConstraintSuspension) {
			if (WheelIdx < ConstraintHandles.Num()) {
				FPhysicsConstraintHandle& ConstraintHandle = ConstraintHandles[WheelIdx];
				if (ConstraintHandle.IsValid()) {
					if (Chaos::FSuspensionConstraint* Constraint = static_cast<Chaos::FSuspensionConstraint*>(ConstraintHandle.Constraint)) {
						if (Chaos::FSuspensionConstraintPhysicsProxy* Proxy = Constraint->GetProxy<Chaos::FSuspensionConstraintPhysicsProxy>()) {
							const Chaos::FVec3 TargetPos = HitResult.ImpactPoint + (PWheel.GetEffectiveRadius() * VehicleState.VehicleUpAxis);

							Chaos::FPhysicsSolver* Solver = Proxy->GetSolver<Chaos::FPhysicsSolver>();

							Solver->SetSuspensionTarget(Constraint, TargetPos, HitResult.ImpactNormal, PWheel.InContact());
						}
					}
				}
			}
		}

		if (PWheel.InContact()) {
			NewDesiredLength = HitResult.Distance;

			SuspensionMovePosition = -FVector::DotProduct(WheelState.WheelWorldLocation[WheelIdx] - HitResult.ImpactPoint, VehicleState.VehicleUpAxis) + PWheel.GetEffectiveRadius();

			PSuspension.SetSuspensionLength(NewDesiredLength, PWheel.GetEffectiveRadius());
			PSuspension.SetLocalVelocity(WheelState.LocalWheelVelocity[WheelIdx]);
			PSuspension.Simulate(DeltaTime);

			float ForceMagnitude = PSuspension.GetSuspensionForce();

			FVector GroundZVector = HitResult.Normal;
			FVector SuspensionForceVector = VehicleState.VehicleUpAxis * ForceMagnitude;

			FVector SusApplicationPoint = WheelState.WheelWorldLocation[WheelIdx] + PVehicle->Suspension[WheelIdx].Setup().SuspensionForceOffset;

			check(PWheel.InContact());
			if (GWheeledVehicleDebugParams.DisableConstraintSuspension) {
				AddForceAtPosition(SuspensionForceVector, SusApplicationPoint);
			}

			ForceMagnitude = PSuspension.Setup().WheelLoadRatio * ForceMagnitude + (1.f - PSuspension.Setup().WheelLoadRatio) * PSuspension.Setup().RestingForce;
			PWheel.SetWheelLoadForce(ForceMagnitude);
			PWheel.SetMassPerWheel(RigidHandle->M() / PVehicle->Wheels.Num());
			SusForces[WheelIdx] = ForceMagnitude;

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
			if (GWheeledVehicleDebugParams.ShowSuspensionForces) {
				Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(SusApplicationPoint, SusApplicationPoint + SuspensionForceVector * GVehicleDebugParams.ForceDebugScaling, FColor::Blue, false, -1.0f, 0, 5);

				Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(SusApplicationPoint, SusApplicationPoint + GroundZVector * 140.f, FColor::Yellow, false, -1.0f, 0, 5);
			}
#endif
		}
		else {
			PSuspension.SetSuspensionLength(PSuspension.GetTraceLength(PWheel.GetEffectiveRadius()), PWheel.Setup().WheelRadius);
			PWheel.SetWheelLoadForce(0.f);
		}
	}

	if (!GWheeledVehicleDebugParams.DisableRollbarForces) {
		for (auto& Axle : PVehicle->GetAxles()) {
			//#todo: only works with 2 wheels on an axle at present
			if (Axle.Setup.WheelIndex.Num() == 2) {
				uint16 WheelIdxA = Axle.Setup.WheelIndex[0];
				uint16 WheelIdxB = Axle.Setup.WheelIndex[1];

				float FV = Axle.Setup.RollbarScaling;
				float ForceDiffOnAxleF = SusForces[WheelIdxA] - SusForces[WheelIdxB];
				FVector ForceVector0 = VehicleState.VehicleUpAxis * ForceDiffOnAxleF * FV;
				FVector ForceVector1 = VehicleState.VehicleUpAxis * ForceDiffOnAxleF * -FV;

				FVector SusApplicationPoint0 = WheelState.WheelWorldLocation[WheelIdxA] + PVehicle->Suspension[WheelIdxA].Setup().SuspensionForceOffset;
				AddForceAtPosition(ForceVector0, SusApplicationPoint0);

				FVector SusApplicationPoint1 = WheelState.WheelWorldLocation[WheelIdxB] + PVehicle->Suspension[WheelIdxB].Setup().SuspensionForceOffset;
				AddForceAtPosition(ForceVector1, SusApplicationPoint1);
			}
		}
	}
}

void UChaosWheeledVehicleSimulation::ProcessSteering(const FControlInputs& ControlInputs) {
	auto& PSteering = PVehicle->GetSteering();

	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++) {
		auto& PWheel = PVehicle->Wheels[WheelIdx]; // Physics Wheel
		FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

		if (PWheel.SteeringEnabled) {
			float SpeedScale = 1.0f;

			// allow full counter steering when steering into a power slide
			//if (ControlInputs.SteeringInput * VehicleState.VehicleLocalVelocity.Y > 0.0f)
			{
				SpeedScale = PVehicle->GetSteering().GetSteeringFromVelocity(Chaos::CmSToMPH(VehicleState.ForwardSpeed));
			}

			float SteeringAngle = ControlInputs.SteeringInput * SpeedScale;

			if (FMath::Abs(GWheeledVehicleDebugParams.SteeringOverride) > 0.01f) {
				SteeringAngle = PWheel.MaxSteeringAngle * GWheeledVehicleDebugParams.SteeringOverride;
			}
			else {
				float WheelSide = PVehicle->GetSuspension(WheelIdx).GetLocalRestingPosition().Y;
				SteeringAngle = PSteering.GetSteeringAngle(SteeringAngle, PWheel.MaxSteeringAngle, WheelSide);
			}

			PWheel.SetSteeringAngle(SteeringAngle);
		}
		else {
			PWheel.SetSteeringAngle(0.0f);
		}
	}
}

void UChaosWheeledVehicleSimulation::ApplyInput(const FControlInputs& ControlInputs, float DeltaTime) {
	UChaosVehicleSimulation::ApplyInput(ControlInputs, DeltaTime);

	FControlInputs ModifiedInputs = ControlInputs;

	float EngineBraking = 0.f;
	if (PVehicle->HasTransmission() && PVehicle->HasEngine()) {
		auto& PEngine = PVehicle->GetEngine();
		auto& PTransmission = PVehicle->GetTransmission();

		if (ModifiedInputs.TransmissionType != PTransmission.Setup().TransmissionType) {
			PTransmission.AccessSetup().TransmissionType = ModifiedInputs.TransmissionType;
		}

		if (ModifiedInputs.GearUpInput) {
			PTransmission.ChangeUp();
			ModifiedInputs.GearUpInput = false;
		}

		if (ModifiedInputs.GearDownInput) {
			PTransmission.ChangeDown();
			ModifiedInputs.GearDownInput = false;
		}

		if (GWheeledVehicleDebugParams.ThrottleOverride > 0.f) {
			PTransmission.SetGear(1, true);
			ModifiedInputs.BrakeInput = 0.f;
			PEngine.SetThrottle(GWheeledVehicleDebugParams.ThrottleOverride);
		}
		else {
			PEngine.SetThrottle(ModifiedInputs.ThrottleInput * ModifiedInputs.ThrottleInput);
		}

		EngineBraking = PEngine.GetEngineRPM() * PEngine.Setup().EngineBrakeEffect;
	}

	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++) {
		auto& PWheel = PVehicle->Wheels[WheelIdx];

		float EngineBrakingForce = 0.0f;
		if ((ModifiedInputs.ThrottleInput < SMALL_NUMBER) && FMath::Abs(VehicleState.ForwardSpeed) > SMALL_NUMBER && PWheel.EngineEnabled) {
			EngineBrakingForce = EngineBraking;
		}

		if (PWheel.BrakeEnabled) {
			float BrakeForce = PWheel.MaxBrakeTorque * ModifiedInputs.BrakeInput;
			PWheel.SetBrakeTorque(Chaos::TorqueMToCm(BrakeForce + EngineBrakingForce), FMath::Abs(EngineBrakingForce) > FMath::Abs(BrakeForce));
		}
		else {
			PWheel.SetBrakeTorque(Chaos::TorqueMToCm(EngineBraking), true);
		}

		if ((ModifiedInputs.HandbrakeInput && PWheel.HandbrakeEnabled)) {
			float HandbrakeForce = ModifiedInputs.HandbrakeInput * PWheel.HandbrakeTorque;
			PWheel.SetBrakeTorque(Chaos::TorqueMToCm(HandbrakeForce));
		}
	}
}

bool UChaosWheeledVehicleSimulation::IsWheelSpinning() const {
	for (auto& Wheel : PVehicle->Wheels) {
		if (Wheel.IsSlipping()) {
			return true;
		}
	}

	return false;
}

void UChaosWheeledVehicleSimulation::ProcessMechanicalSimulation(float DeltaTime) {
	if (PVehicle->HasEngine()) {
		auto& PEngine = PVehicle->GetEngine();
		auto& PTransmission = PVehicle->GetTransmission();
		auto& PDifferential = PVehicle->GetDifferential();

		float WheelRPM = 0;
		for (int I = 0; I < PVehicle->Wheels.Num(); I++) {
			if (PVehicle->Wheels[I].EngineEnabled) {
				WheelRPM = FMath::Abs(PVehicle->Wheels[I].GetWheelRPM());
			}
		}

		float WheelSpeedRPM = FMath::Abs(PTransmission.GetEngineRPMFromWheelRPM(WheelRPM));
		PEngine.SetEngineRPM(PTransmission.IsOutOfGear(), PTransmission.GetEngineRPMFromWheelRPM(WheelRPM));
		PEngine.Simulate(DeltaTime);

		PTransmission.SetEngineRPM(PEngine.GetEngineRPM());
		// needs engine RPM to decide when to change gear (automatic gearbox)
		PTransmission.SetAllowedToChangeGear(!VehicleState.bVehicleInAir && !IsWheelSpinning());
		float GearRatio = PTransmission.GetGearRatio(PTransmission.GetCurrentGear());

		PTransmission.Simulate(DeltaTime);

		float TransmissionTorque = PTransmission.GetTransmissionTorque(PEngine.GetEngineTorque());
		if (WheelSpeedRPM > PEngine.Setup().MaxRPM) {
			TransmissionTorque = 0.f;
		}

		// apply drive torque to wheels
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++) {
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			if (PWheel.Setup().EngineEnabled) {
				PWheel.SetDriveTorque(Chaos::TorqueMToCm(TransmissionTorque) * PWheel.Setup().TorqueRatio);
			}
			else {
				PWheel.SetDriveTorque(0.f);
			}
		}
	}
}

void UChaosWheeledVehicleSimulation::DrawDebug3D() {
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)

	UChaosVehicleSimulation::DrawDebug3D();

	if (PVehicle == nullptr) {
		return;
	}

	const FTransform BodyTransform = VehicleState.VehicleWorldTransform;

	if (GWheeledVehicleDebugParams.ShowSuspensionLimits) {
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++) {
			auto& PSuspension = PVehicle->Suspension[WheelIdx];
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			// push the visualization out a bit sideways from the wheel model so we can actually see it
			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 48.0f;
			const FVector& WheelOffset = PSuspension.GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f) {
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector LocalDirection = PSuspension.Setup().SuspensionAxis;
			FVector WorldLocation = BodyTransform.TransformPosition(WheelOffset);
			FVector WorldDirection = BodyTransform.TransformVector(LocalDirection);

			FVector Start = WorldLocation + WorldDirection * (PWheel.GetEffectiveRadius() - PSuspension.Setup().SuspensionMaxRaise);
			FVector End = WorldLocation + WorldDirection * (PWheel.GetEffectiveRadius() + PSuspension.Setup().SuspensionMaxDrop);

			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(Start + VehicleRightAxis, End + VehicleRightAxis, FColor::Orange, false, -1.f, 0, 3.f);

			FVector Start2 = WorldLocation - WorldDirection * PSuspension.Setup().SuspensionMaxRaise;
			FVector End2 = WorldLocation + WorldDirection * PSuspension.Setup().SuspensionMaxDrop;

			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(Start2 + VehicleRightAxis, End2 + VehicleRightAxis, FColor::Yellow, false, -1.f, 0, 3.f);
		}
	}

	if (GWheeledVehicleDebugParams.ShowRaycastComponent) {
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++) {
			FHitResult& Hit = WheelState.TraceResult[WheelIdx];

			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 20.0f;
			FVector VehicleUpAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Z) * 20.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f) {
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector Pt = Hit.ImpactPoint + VehicleRightAxis;
			if (Hit.GetComponent()) {
				Chaos::FDebugDrawQueue::GetInstance().DrawDebugString(Pt + VehicleRightAxis, Hit.GetComponent()->GetName(), nullptr, FColor::White, -1.f, true, 1.0f);
			}
		}
	}

	if (GWheeledVehicleDebugParams.ShowRaycastMaterial) {
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++) {
			FHitResult& Hit = WheelState.TraceResult[WheelIdx];

			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 20.0f;
			FVector VehicleUpAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Z) * 20.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f) {
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector Pt = Hit.ImpactPoint + VehicleRightAxis;
			if (Hit.PhysMaterial.IsValid()) {
				Chaos::FDebugDrawQueue::GetInstance().DrawDebugString(Pt + VehicleRightAxis + VehicleUpAxis, Hit.PhysMaterial->GetName(), nullptr, FColor::White, -1.f, true, 1.0f);
			}
		}
	}

	if (GWheeledVehicleDebugParams.ShowWheelCollisionNormal) {
		FString Name;
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++) {
			FHitResult& Hit = WheelState.TraceResult[WheelIdx];

			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 20.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f) {
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector Pt = Hit.ImpactPoint + VehicleRightAxis;
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(Pt, Pt + Hit.Normal * 20.0f, FColor::Yellow, false, 1.0f, 0, 1.0f);
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugSphere(Pt, 5.0f, 4, FColor::White, false, 1.0f, 0, 1.0f);
		}
	}

	if (GWheeledVehicleDebugParams.ShowSuspensionRaycasts) {
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++) {
			const FVector& TraceStart = WheelState.Trace[WheelIdx].Start;
			const FVector& TraceEnd = WheelState.Trace[WheelIdx].End;

			// push the visualization out a bit sideways from the wheel model so we can actually see it
			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 50.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f) {
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FColor UseColor = PVehicle->Wheels[WheelIdx].InContact() ? FColor::Green : FColor::Red;
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugDirectionalArrow(TraceStart + VehicleRightAxis, TraceEnd + VehicleRightAxis, 10.f, UseColor, false, -1.f, 0, 2.f);

			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(TraceStart, TraceStart + VehicleRightAxis, FColor::White, false, -1.f, 0, 1.f);
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(TraceEnd, TraceEnd + VehicleRightAxis, FColor::White, false, -1.f, 0, 1.f);
		}
	}
#endif
}

void UChaosWheeledVehicleSimulation::FillOutputState(FWheeledVehicleAsyncOutput& Output) {
	// #Note: remember to copy/interpolate values from the physics thread output in UChaosVehicleMovementComponent::ParallelUpdate
	const auto& VehicleWheels = PVehicle->Wheels;
	auto& VehicleSuspension = PVehicle->Suspension;
	if (PVehicle->HasTransmission()) {
		auto& Transmission = PVehicle->GetTransmission();
		Output.VehicleSimOutput.CurrentGear = Transmission.GetCurrentGear();
		Output.VehicleSimOutput.TargetGear = Transmission.GetTargetGear();
		Output.VehicleSimOutput.TransmissionRPM = Transmission.GetTransmissionRPM();
		Output.VehicleSimOutput.TransmissionTorque = Transmission.GetTransmissionTorque(PVehicle->GetEngine().GetTorqueFromRPM(false));
	}
	if (PVehicle->HasEngine()) {
		auto& Engine = PVehicle->GetEngine();
		Output.VehicleSimOutput.EngineRPM = Engine.GetEngineRPM();
		Output.VehicleSimOutput.EngineTorque = Engine.GetEngineTorque();
	}

	// #TODO: can we avoid copies when async is turned off
	for (int WheelIdx = 0; WheelIdx < VehicleWheels.Num(); WheelIdx++) {
		FWheelsOutput WheelsOut;
		WheelsOut.InContact = VehicleWheels[WheelIdx].InContact();
		WheelsOut.SteeringAngle = VehicleWheels[WheelIdx].GetSteeringAngle();
		WheelsOut.AngularPosition = VehicleWheels[WheelIdx].GetAngularPosition();
		WheelsOut.AngularVelocity = VehicleWheels[WheelIdx].GetAngularVelocity();
		WheelsOut.WheelRadius = VehicleWheels[WheelIdx].GetEffectiveRadius();

		WheelsOut.LateralAdhesiveLimit = VehicleWheels[WheelIdx].LateralAdhesiveLimit;
		WheelsOut.LongitudinalAdhesiveLimit = VehicleWheels[WheelIdx].LongitudinalAdhesiveLimit;

		WheelsOut.bIsSlipping = VehicleWheels[WheelIdx].IsSlipping();
		WheelsOut.SlipMagnitude = VehicleWheels[WheelIdx].GetSlipMagnitude();
		WheelsOut.bIsSkidding = VehicleWheels[WheelIdx].IsSkidding();
		WheelsOut.SkidMagnitude = VehicleWheels[WheelIdx].GetSkidMagnitude();
		WheelsOut.SkidNormal = WheelState.WorldWheelVelocity[WheelIdx].GetSafeNormal();
		WheelsOut.SlipAngle = VehicleWheels[WheelIdx].GetSlipAngle();

		WheelsOut.SuspensionOffset = VehicleSuspension[WheelIdx].GetSuspensionOffset();
		WheelsOut.SpringForce = VehicleSuspension[WheelIdx].GetSuspensionForce();
		WheelsOut.NormalizedSuspensionLength = VehicleSuspension[WheelIdx].GetNormalizedLength();

		WheelsOut.bBlockingHit = WheelState.TraceResult[WheelIdx].bBlockingHit;
		WheelsOut.ImpactPoint = WheelState.TraceResult[WheelIdx].ImpactPoint;
		WheelsOut.PhysMaterial = WheelState.TraceResult[WheelIdx].PhysMaterial;

		Output.VehicleSimOutput.Wheels.Add(WheelsOut);
	}
}

void UChaosWheeledVehicleSimulation::UpdateConstraintHandles(TArray<FPhysicsConstraintHandle>& ConstraintHandlesIn) {
	UChaosVehicleSimulation::UpdateConstraintHandles(ConstraintHandlesIn);
	ConstraintHandles = ConstraintHandlesIn;
}

/**
 * UChaosWheeledVehicleMovementComponent
 */
UChaosWheeledVehicleMovementComponent::UChaosWheeledVehicleMovementComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer) {
	// default values setup

	EngineSetup.InitDefaults();
	DifferentialSetup.InitDefaults();
	TransmissionSetup.InitDefaults();
	SteeringSetup.InitDefaults();

	// It's possible to switch whole systems off if they are not required
	bMechanicalSimEnabled = true;
	bSuspensionEnabled = true;
	bWheelFrictionEnabled = true;

	// new vehicles don't use legacy method where friction forces are applied at wheel rather than wheel contact point 
	bLegacyWheelFrictionPosition = false;

	WheelTraceCollisionResponses = FCollisionResponseContainer::GetDefaultResponseContainer();
	WheelTraceCollisionResponses.Vehicle = ECR_Ignore;
}

// Public
void UChaosWheeledVehicleMovementComponent::Serialize(FArchive& Ar) {
	Super::Serialize(Ar);

	Ar.UsingCustomVersion(FUE5MainStreamObjectVersion::GUID);
}

void UChaosWheeledVehicleMovementComponent::PostLoad() {
	Super::PostLoad();

#if WITH_EDITORONLY_DATA

	if (GetLinkerCustomVersion(FUE5MainStreamObjectVersion::GUID) < FUE5MainStreamObjectVersion::VehicleFrictionForcePositionChange) {
		bLegacyWheelFrictionPosition = true;
	}

#endif  // #if WITH_EDITORONLY_DATA
}

void UChaosWheeledVehicleMovementComponent::UpdateGroundedState(float DeltaTime) {
	int NumWheels = 0;
	if (PVehicleOutput) {
		for (int WheelIdx = 0; WheelIdx < PVehicleOutput->Wheels.Num(); WheelIdx++) {
			if (PVehicleOutput->Wheels[WheelIdx].InContact) {
				VehicleState.NumWheelsOnGround++;
			}
			else {
				VehicleState.bVehicleInAir = true;
			}
			NumWheels++;
		}
	}
	VehicleState.bAllWheelsOnGround = (VehicleState.NumWheelsOnGround == NumWheels);
}

#if WITH_EDITOR
void UChaosWheeledVehicleMovementComponent::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) {
	const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	RecalculateAxles();

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

void UChaosWheeledVehicleMovementComponent::FixupSkeletalMesh() {
	Super::FixupSkeletalMesh();

	if (USkeletalMeshComponent* Mesh = Cast<USkeletalMeshComponent>(GetMesh())) {
		if (UPhysicsAsset* PhysicsAsset = Mesh->GetPhysicsAsset()) {
			for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx) {
				FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];
				if (WheelSetup.BoneName != NAME_None) {
					int32 BodySetupIdx = PhysicsAsset->FindBodyIndex(WheelSetup.BoneName);

					if (BodySetupIdx >= 0 && (BodySetupIdx < Mesh->Bodies.Num())) {
						FBodyInstance* BodyInstanceWheel = Mesh->Bodies[BodySetupIdx];
						BodyInstanceWheel->SetResponseToAllChannels(ECR_Ignore);
						//turn off collision for wheel automatically

						if (UBodySetup* BodySetup = PhysicsAsset->SkeletalBodySetups[BodySetupIdx]) {
							{
								BodyInstanceWheel->SetInstanceSimulatePhysics(false);
								//BodyInstanceWheel->SetCollisionEnabled(ECollisionEnabled::NoCollision);
							}

							bool DeleteOriginalWheelConstraints = true;
							if (DeleteOriginalWheelConstraints) {
								//and get rid of constraints on the wheels. TODO: right now we remove all wheel constraints, we probably only want to remove parent constraints
								TArray<int32> WheelConstraints;
								PhysicsAsset->BodyFindConstraints(BodySetupIdx, WheelConstraints);
								for (int32 ConstraintIdx = 0; ConstraintIdx < WheelConstraints.Num(); ++ConstraintIdx) {
									FConstraintInstance* ConInst = Mesh->Constraints[WheelConstraints[ConstraintIdx]];
									ConInst->TermConstraint();
								}
							}
						}
					}

					if (!GWheeledVehicleDebugParams.DisableConstraintSuspension) {
						FBodyInstance* TargetInstance = UpdatedPrimitive->GetBodyInstance();
						if (TargetInstance) {
							FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
								const FVector LocalWheel = GetWheelRestingPosition(WheelSetup);
								FPhysicsConstraintHandle ConstraintHandle = FPhysicsInterface::CreateSuspension(Chassis, LocalWheel);

								if (ConstraintHandle.IsValid()) {
									UChaosVehicleWheel* Wheel = Wheels[WheelIdx];
									check(Wheel);
									ConstraintHandles.Add(ConstraintHandle);
									if (Chaos::FSuspensionConstraint* Constraint = static_cast<Chaos::FSuspensionConstraint*>(ConstraintHandle.Constraint)) {
										Constraint->SetHardstopStiffness(1.0f);
										Constraint->SetSpringStiffness(Chaos::MToCm(Wheel->SpringRate) * 0.25f);
										Constraint->SetSpringPreload(Chaos::MToCm(Wheel->SpringPreload));
										Constraint->SetSpringDamping(Wheel->SuspensionDampingRatio * 5.0f);
										Constraint->SetMinLength(-Wheel->SuspensionMaxRaise);
										Constraint->SetMaxLength(Wheel->SuspensionMaxDrop);
										Constraint->SetAxis(-Wheel->SuspensionAxis);
									}
								}
							});
						}
					}
				}
			}
		}

		VehicleSimulationPT->UpdateConstraintHandles(ConstraintHandles);
		// TODO: think of a better way to communicate this data

		Mesh->KinematicBonesUpdateType = EKinematicBonesUpdateToPhysics::SkipSimulatingBones;
	}
}


bool UChaosWheeledVehicleMovementComponent::CanCreateVehicle() const {
	if (!Super::CanCreateVehicle()) return false;

	check(GetOwner());
	FString ActorName = GetOwner()->GetName();

	for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx) {
		const FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];

		if (WheelSetup.WheelClass == NULL) {
			UE_LOG(LogVehicle, Warning, TEXT("Can't create vehicle %s (%s). Wheel %d is not set."), *ActorName, *GetPathName(), WheelIdx);
			return false;
		}

		if (WheelSetup.BoneName == NAME_None) {
			UE_LOG(LogVehicle, Warning, TEXT("Can't create vehicle %s (%s). Bone name for wheel %d is not set."), *ActorName, *GetPathName(), WheelIdx);
			return false;
		}
	}

	return true;
}


void UChaosWheeledVehicleMovementComponent::OnCreatePhysicsState() {
	Super::OnCreatePhysicsState();

	VehicleSetupTag = FChaosVehicleManager::VehicleSetupTag;

	// only create Physics vehicle in game
	UWorld* World = GetWorld();
	if (World->IsGameWorld()) {
		FPhysScene* PhysScene = World->GetPhysicsScene();

		if (PhysScene && FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene)) {
			CreateVehicle();
			FixupSkeletalMesh();

			if (HasValidPhysicsState()) {
				FChaosVehicleManager* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene);
				VehicleManager->AddVehicle(StaticClass()->GetName(), this);
			}
		}
	}

	FBodyInstance* BodyInstance = nullptr;
	if (USkeletalMeshComponent* SkeletalMesh = GetSkeletalMesh()) {
		SkeletalMesh->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::OnlyTickPoseWhenRendered;
		BodyInstance = &SkeletalMesh->BodyInstance;
	}
}

void UChaosWheeledVehicleMovementComponent::CreateVehicle() {
	ComputeConstants();
	{
		if (CanCreateVehicle()) {
			check(UpdatedComponent);
			if (ensure(UpdatedPrimitive != nullptr)) {
				auto PVehicle = CreatePhysicsVehicle();

				// Low level physics representation
				SetupVehicle(PVehicle);

				if (HasValidPhysicsState()) {
					PostSetupVehicle();
				}

				// Physics thread simulation class will now take ownership of the PVehicle pointer, we cannot safely use it anymore from the game thread
				VehicleSimulationPT->Init(PVehicle);
			}

			VehicleState.CaptureState(GetBodyInstance(), GetGravityZ(), 0.01667f);
		}
	}

	if (PVehicleOutput) {
		CreateWheels();

		// Need to bind to the notify delegate on the mesh in case physics state is changed
		if (USkeletalMeshComponent* MeshComp = GetSkeletalMesh()) {
			MeshOnPhysicsStateChangeHandle = MeshComp->RegisterOnPhysicsCreatedDelegate(FOnSkelMeshPhysicsCreated::CreateUObject(this, &UActorComponent::RecreatePhysicsState));
			if (UVehicleAnimationInstance* VehicleAnimInstance = Cast<UVehicleAnimationInstance>(MeshComp->GetAnimInstance())) {
				VehicleAnimInstance->SetWheeledVehicleComponent(this);
			}
		}
	}
}

void UChaosWheeledVehicleMovementComponent::OnDestroyPhysicsState() {
	if (PVehicleOutput.IsValid()) {
		if (MeshOnPhysicsStateChangeHandle.IsValid()) {
			if (USkeletalMeshComponent* MeshComp = GetSkeletalMesh()) {
				MeshComp->UnregisterOnPhysicsCreatedDelegate(MeshOnPhysicsStateChangeHandle);
			}
		}

		DestroyWheels();

		if (ConstraintHandles.Num() > 0) {
			for (FPhysicsConstraintHandle ConstraintHandle : ConstraintHandles) {
				FPhysicsCommand::ExecuteWrite(ConstraintHandle, [&](const FPhysicsConstraintHandle& Constraint) {
					FPhysicsInterface::ReleaseConstraint(ConstraintHandle);
				});
			}
		}
		ConstraintHandles.Empty();
	}
	Super::OnDestroyPhysicsState();
}

void UChaosWheeledVehicleMovementComponent::SetTargetGear(int32 GearNum, bool bImmediate) {
	if (HasValidPhysicsState() && GearNum != PVehicleOutput->TargetGear) {
		FBodyInstance* TargetInstance = UpdatedPrimitive->GetBodyInstance();

		if (TargetInstance) {
			FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
				if (GetVehicleSimulationPT() && GetVehicleSimulationPT()->GetPVehicle() && GetVehicleSimulationPT()->GetPVehicle()->HasTransmission()) {
					GetVehicleSimulationPT()->GetPVehicle()->GetTransmission().SetGear(GearNum, bImmediate);
				}
			});
		}

		TargetGear = GearNum;
	}
}

int32 UChaosWheeledVehicleMovementComponent::GetCurrentGear() const {
	return (HasValidPhysicsState()) ? PVehicleOutput->CurrentGear : 0;
}

void UChaosWheeledVehicleMovementComponent::ClearRawInput() {
	Super::ClearRawInput();

	// Send this immediately.
	int32 CurrentGear = 0;
	if (HasValidPhysicsState()) {
		CurrentGear = PVehicleOutput->CurrentGear;
	}

	AController* Controller = GetController();
	if (Controller && Controller->IsLocalController() && HasValidPhysicsState()) {
		ServerUpdateState(SteeringInput, ThrottleInput, BrakeInput, HandbrakeInput, CurrentGear);
	}
}


void UChaosWheeledVehicleMovementComponent::NextDebugPage() {
	int PageAsInt = (int)DebugPage;
	PageAsInt++;
	if (PageAsInt >= EDebugPages::MaxDebugPages) {
		PageAsInt = 0;
	}
	DebugPage = (EDebugPages)PageAsInt;
}

void UChaosWheeledVehicleMovementComponent::PrevDebugPage() {
	int PageAsInt = (int)DebugPage;
	PageAsInt--;
	if (PageAsInt < 0) {
		PageAsInt = EDebugPages::MaxDebugPages - 1;
	}
	DebugPage = (EDebugPages)PageAsInt;
}


// Setup
void UChaosWheeledVehicleMovementComponent::ComputeConstants() {
	Super::ComputeConstants();
}

void UChaosWheeledVehicleMovementComponent::CreateWheels() {
	// Wheels num is getting copied when blueprint recompiles, so we have to manually reset here
	Wheels.Reset();

	// Instantiate the wheels
	for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx) {
		UChaosVehicleWheel* Wheel = NewObject<UChaosVehicleWheel>(this, WheelSetups[WheelIdx].WheelClass);
		check(Wheel);

		Wheels.Add(Wheel);
	}

	// Initialize the wheels
	for (int32 WheelIdx = 0; WheelIdx < Wheels.Num(); ++WheelIdx) {
		Wheels[WheelIdx]->Init(this, WheelIdx);
	}

	WheelStatus.SetNum(WheelSetups.Num());
	CachedState.SetNum(WheelSetups.Num());

	RecalculateAxles();
}

void UChaosWheeledVehicleMovementComponent::DestroyWheels() {
	for (int32 i = 0; i < Wheels.Num(); ++i) {
		Wheels[i]->Shutdown();
	}

	Wheels.Reset();
}

void UChaosWheeledVehicleMovementComponent::SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle) {
	check(PVehicle);

	Chaos::FSimpleAerodynamicsSim AerodynamicsSim(&GetAerodynamicsConfig());
	PVehicle->Aerodynamics.Add(AerodynamicsSim);

	for (FVehicleAerofoilConfig& AerofoilSetup : Aerofoils) {
		Chaos::FAerofoil AerofoilSim(&AerofoilSetup.GetPhysicsAerofoilConfig(*this));
		PVehicle->Aerofoils.Add(AerofoilSim);
	}

	for (FVehicleThrustConfig& ThrustSetup : Thrusters) {
		Chaos::FSimpleThrustSim ThrustSim(&ThrustSetup.GetPhysicsThrusterConfig(*this));
		PVehicle->Thrusters.Add(ThrustSim);
	}
	NumDrivenWheels = 0;

	// we are allowed any number of wheels not limited to only 4
	for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx) {
		UChaosVehicleWheel* Wheel = WheelSetups[WheelIdx].WheelClass.GetDefaultObject();

		// create Dynamic states passing in pointer to their Static setup data
		Chaos::FSimpleWheelSim WheelSim(&Wheel->GetPhysicsWheelConfig());

		if (Wheel->GetAxleType() != EAxleType::Undefined) {
			bool EngineEnable = false;
			if (Wheel->GetAxleType() == EAxleType::Front) {
				if (DifferentialSetup.DifferentialType == EVehicleDifferential::AllWheelDrive || DifferentialSetup.DifferentialType == EVehicleDifferential::FrontWheelDrive) {
					EngineEnable = true;
				}
			}
			else if (Wheel->GetAxleType() == EAxleType::Rear) {
				if (DifferentialSetup.DifferentialType == EVehicleDifferential::AllWheelDrive || DifferentialSetup.DifferentialType == EVehicleDifferential::RearWheelDrive) {
					EngineEnable = true;
				}
			}

			WheelSim.AccessSetup().EngineEnabled = EngineEnable;
		}

		WheelSim.SetWheelRadius(Wheel->WheelRadius); // initial radius
		PVehicle->Wheels.Add(WheelSim);

		FWheelsOutput WheelsOutput; // Receptacle for Data coming out of physics simulation on physics thread
		PVehicleOutput->Wheels.Add(WheelsOutput);

		Chaos::FSimpleSuspensionSim SuspensionSim(&Wheel->GetPhysicsSuspensionConfig());
		PVehicle->Suspension.Add(SuspensionSim);

		if (WheelSim.Setup().EngineEnabled) {
			NumDrivenWheels++;
		}

		// for debugging to identify a single wheel
		PVehicle->Wheels[WheelIdx].SetWheelIndex(WheelIdx);
		PVehicle->Suspension[WheelIdx].SetSpringIndex(WheelIdx);
		PVehicle->NumDrivenWheels = NumDrivenWheels;

		PVehicle->bLegacyWheelFrictionPosition = bLegacyWheelFrictionPosition;
	}

	RecalculateAxles();

	// setup axles in PVehicle
	for (auto& Axle : AxleToWheelMap) {
		TArray<int>& WheelIndices = Axle.Value;

		FChaosWheelSetup& WheelSetup = WheelSetups[WheelIndices[0]];
		UChaosVehicleWheel* WheelData = WheelSetup.WheelClass.GetDefaultObject();

		Chaos::FAxleSim AxleSim;
		AxleSim.Setup.RollbarScaling = WheelData->RollbarScaling;

		for (int WheelIdx : WheelIndices) {
			AxleSim.Setup.WheelIndex.Add(WheelIdx);
		}

		PVehicle->Axles.Add(AxleSim);
	}

	// cache this value as it's useful for steering setup calculations and debug rendering
	WheelTrackDimensions = CalculateWheelLayoutDimensions();

	if (EngineSetup.TorqueCurve.GetRichCurve()->IsEmpty()) {
		FString ActorName = "Unknown";
		if (GetOwner()) {
			ActorName = GetOwner()->GetName();
		}
		UE_LOG(LogVehicle, Warning, TEXT("Vehicle %s has no torque curve defined, disabling mechanical simulation."), *ActorName);

		bMechanicalSimEnabled = false;
	}

	if (bMechanicalSimEnabled) {
		Chaos::FSimpleEngineSim EngineSim(&EngineSetup.GetPhysicsEngineConfig());
		PVehicle->Engine.Add(EngineSim);

		Chaos::FSimpleTransmissionSim TransmissionSim(&TransmissionSetup.GetPhysicsTransmissionConfig());
		PVehicle->Transmission.Add(TransmissionSim);
		TransmissionType = TransmissionSim.Setup().TransmissionType;
		// current transmission mode - dynamically modifiable at runtime

		Chaos::FSimpleDifferentialSim DifferentialSim(&DifferentialSetup.GetPhysicsDifferentialConfig());
		PVehicle->Differential.Add(DifferentialSim);

		// Setup override of wheel TorqueRatio & EngineEnabled from vehicle differential settings
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++) {
			Chaos::FSimpleWheelSim& PWheel = PVehicle->Wheels[WheelIdx];
			bool IsWheelPowered = Chaos::FTransmissionUtility::IsWheelPowered(DifferentialSim.Setup().DifferentialType, PWheel.Setup().AxleType, PWheel.EngineEnabled);
			PWheel.AccessSetup().EngineEnabled = IsWheelPowered;
			PWheel.EngineEnabled = IsWheelPowered;

			float TorqueRatio = Chaos::FTransmissionUtility::GetTorqueRatioForWheel(DifferentialSim, WheelIdx, PVehicle->Wheels);
			PWheel.AccessSetup().TorqueRatio = TorqueRatio;
		}
	}

	Chaos::FSimpleSteeringSim SteeringSim(&SteeringSetup.GetPhysicsSteeringConfig(WheelTrackDimensions));
	PVehicle->Steering.Add(SteeringSim);

	Chaos::FTorqueControlSim TorqueSim(&TorqueControl.GetTorqueControlConfig());
	PVehicle->TorqueControlSim.Add(TorqueSim);

	Chaos::FTargetRotationControlSim TargetRotationSim(&TargetRotationControl.GetTargetRotationControlConfig());
	PVehicle->TargetRotationControlSim.Add(TargetRotationSim);

	Chaos::FStabilizeControlSim StabilizeSim(&StabilizeControl.GetStabilizeControlConfig());
	PVehicle->StabilizeControlSim.Add(StabilizeSim);

	// Setup the chassis and wheel shapes
	SetupVehicleShapes();

	// Setup mass properties
	SetupVehicleMass();

	// Setup Suspension
	SetupSuspension(PVehicle);
}

void UChaosWheeledVehicleMovementComponent::ResetVehicleState() {
	UChaosVehicleMovementComponent::ResetVehicleState();

	for (FWheelStatus& WheelInfo : WheelStatus) {
		WheelInfo.Init();
	}

	for (FCachedState& State : CachedState) {
		State.bIsValid = false;
	}
}

void UChaosWheeledVehicleMovementComponent::SetupVehicleShapes() {
	if (!UpdatedPrimitive) {
		return;
	}
}

void UChaosWheeledVehicleMovementComponent::SetupSuspension(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle) {
	if (!PVehicle->IsValid() || PVehicle->Suspension.Num() == 0) {
		return;
	}

	float TotalMass = this->Mass;
	ensureMsgf(TotalMass >= 1.0f, TEXT("The mass of this vehicle is too small."));

	TArray<FVector> LocalSpringPositions;

	// cache vehicle local position of springs
	for (int SpringIdx = 0; SpringIdx < PVehicle->Suspension.Num(); SpringIdx++) {
		auto& PSuspension = PVehicle->Suspension[SpringIdx];

		PSuspension.AccessSetup().MaxLength = PSuspension.Setup().SuspensionMaxDrop + PSuspension.Setup().SuspensionMaxRaise;

		FVector TotalOffset = GetWheelRestingPosition(WheelSetups[SpringIdx]);
		LocalSpringPositions.Add(TotalOffset);
		PVehicle->Suspension[SpringIdx].SetLocalRestingPosition(LocalSpringPositions[SpringIdx]);
	}

	// Calculate the mass that will rest on each of the springs
	TArray<float> OutSprungMasses;
	if (!LocalSpringPositions.IsEmpty() && !FSuspensionUtility::ComputeSprungMasses(LocalSpringPositions, TotalMass, OutSprungMasses)) {
		// if the sprung mass calc fails fall back to something that will still simulate
		for (int Index = 0; Index < OutSprungMasses.Num(); Index++) {
			OutSprungMasses[Index] = TotalMass / OutSprungMasses.Num();
		}
	}

	// Calculate spring damping values we will use for physics simulation from the normalized damping ratio
	for (int SpringIdx = 0; SpringIdx < PVehicle->Suspension.Num(); SpringIdx++) {
		auto& Susp = PVehicle->Suspension[SpringIdx];
		float NaturalFrequency = FSuspensionUtility::ComputeNaturalFrequency(Susp.Setup().SpringRate, OutSprungMasses[SpringIdx]);
		float Damping = FSuspensionUtility::ComputeDamping(Susp.Setup().SpringRate, OutSprungMasses[SpringIdx], Susp.Setup().DampingRatio);
		UE_LOG(LogChaos, Verbose, TEXT("Spring %d: OutNaturalFrequency %.1f Hz  (@1.0) DampingRate %.1f"), SpringIdx, NaturalFrequency / (2.0f * PI), Damping);

		PVehicle->Suspension[SpringIdx].AccessSetup().ReboundDamping = Damping;
		PVehicle->Suspension[SpringIdx].AccessSetup().CompressionDamping = Damping;
		PVehicle->Suspension[SpringIdx].AccessSetup().RestingForce = OutSprungMasses[SpringIdx] * -GetGravityZ();
	}
}

void UChaosWheeledVehicleMovementComponent::RecalculateAxles() {
	AxleToWheelMap.Empty();

	for (int WheelIdx = 0; WheelIdx < WheelSetups.Num(); WheelIdx++) {
		FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];
		UChaosVehicleWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();

		if (auto* WheelIdxArray = AxleToWheelMap.Find(Wheel)) {
			WheelIdxArray->Add(WheelIdx);
		}
		else {
			TArray<int> WheelIndices;
			WheelIndices.Add(WheelIdx);
			AxleToWheelMap.Add(Wheel, WheelIndices);
		}
	}
}

FVector UChaosWheeledVehicleMovementComponent::GetWheelRestingPosition(const FChaosWheelSetup& WheelSetup) {
	FVector Offset = WheelSetup.WheelClass.GetDefaultObject()->Offset + WheelSetup.AdditionalOffset;
	return LocateBoneOffset(WheelSetup.BoneName, Offset);
}

// Access to data
float UChaosWheeledVehicleMovementComponent::GetEngineRotationSpeed() const {
	float EngineRPM = 0.f;

	if (bMechanicalSimEnabled && PVehicleOutput) {
		EngineRPM = PVehicleOutput->EngineRPM;
	}

	return EngineRPM;
}

float UChaosWheeledVehicleMovementComponent::GetEngineMaxRotationSpeed() const {
	float MaxEngineRPM = 0.f;

	if (bMechanicalSimEnabled) {
		MaxEngineRPM = EngineSetup.MaxRPM;
	}

	return MaxEngineRPM;
}

bool UChaosWheeledVehicleMovementComponent::IsOnAnySurface(const TArray<UPhysicalMaterial*> surfaces) {
	for (auto& wheel : Wheels) {
		if (wheel) {
			if (auto mat = wheel->GetContactSurfaceMaterial()) {
				if (surfaces.Contains(mat)) {
					GEngine->AddOnScreenDebugMessage(-1, 0.1f, FColor::Green, mat->GetName());
					return true;
				}
			}
		}
	}
	return false;
}

bool UChaosWheeledVehicleMovementComponent::HasValidPhysicsState() const {
	return Super::HasValidPhysicsState() && PVehicleOutput.IsValid();
}

// Helper
FVector2D UChaosWheeledVehicleMovementComponent::CalculateWheelLayoutDimensions() {
	FVector2D MaxSize(0.f, 0.f);

	for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx) {
		FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];
		UChaosVehicleWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();
		check(Wheel);

		const FVector WheelOffset = GetWheelRestingPosition(WheelSetup);
		if (FMath::Abs(WheelOffset.Y) > MaxSize.Y) {
			MaxSize.Y = FMath::Abs(WheelOffset.Y);
		}

		if (FMath::Abs(WheelOffset.X) > MaxSize.X) {
			MaxSize.X = FMath::Abs(WheelOffset.X);
		}
	}

	// full width/length not half
	MaxSize *= 2.0f;

	return MaxSize;
}

void UChaosWheeledVehicleMovementComponent::Update(float DeltaTime) {
	UChaosVehicleMovementComponent::Update(DeltaTime);

	if (CurAsyncInput) {
		if (const FBodyInstance* BodyInstance = GetBodyInstance()) {
			if (auto Handle = BodyInstance->ActorHandle) {
				FWheeledVehicleAsyncInput* AsyncInput = static_cast<FWheeledVehicleAsyncInput*>(CurAsyncInput);

				TArray<AActor*> ActorsToIgnore;
				ActorsToIgnore.Add(GetPawnOwner()); // ignore self in scene query

				CurAsyncInput->Proxy = Handle; // vehicles are never static

				AsyncInput->ControlInputs.ThrottleInput = ThrottleInputRate.CalcControlFunction(ThrottleInput);
				AsyncInput->ControlInputs.BrakeInput = BrakeInputRate.CalcControlFunction(BrakeInput);
				AsyncInput->ControlInputs.SteeringInput = SteeringInputRate.CalcControlFunction(SteeringInput);
				AsyncInput->ControlInputs.HandbrakeInput = HandbrakeInput;
				AsyncInput->ControlInputs.GearUpInput = bRawGearUpInput;
				AsyncInput->ControlInputs.GearDownInput = bRawGearDownInput;
				AsyncInput->ControlInputs.TransmissionType = TransmissionType;

				// debug feature to limit the vehicles top speed
				if ((GVehicleDebugParams.SetMaxMPH > 0.f) && (FMath::Abs(ThrottleInput) > 0.0f) && FMath::Abs(GetForwardSpeedMPH()) >= GVehicleDebugParams.SetMaxMPH) {
					AsyncInput->ControlInputs.ThrottleInput = 0.1f;
				}

				AsyncInput->GravityZ = GetGravityZ();
				FCollisionQueryParams TraceParams(NAME_None, FCollisionQueryParams::GetUnknownStatId(), false, nullptr);
				TraceParams.bReturnPhysicalMaterial = true; // we need this to get the surface friction coefficient
				TraceParams.AddIgnoredActors(ActorsToIgnore);
				TraceParams.bTraceComplex = true;
				AsyncInput->TraceParams = TraceParams;
				AsyncInput->TraceCollisionResponse = WheelTraceCollisionResponses;

				AsyncInput->WheelTraceParams.SetNum(Wheels.Num());
				for (int I = 0; I < Wheels.Num(); I++) {
					AsyncInput->WheelTraceParams[I].SweepType = Wheels[I]->SweepType;
					AsyncInput->WheelTraceParams[I].SweepShape = Wheels[I]->SweepShape;
				}
			}
		}
	}
}


// Debug
void UChaosWheeledVehicleMovementComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos) {
	ensure(IsInGameThread());

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)

	Super::DrawDebug(Canvas, YL, YPos);

	FBodyInstance* TargetInstance = GetBodyInstance();

	// #todo: is this rendering multiple times in multiplayer
	if (TargetInstance == nullptr) {
		return;
	}

	float ForwardSpeedMPH = Chaos::CmSToMPH(GetForwardSpeed());

	// always draw this even on (DebugPage == EDebugPages::BasicPage)
	if (bMechanicalSimEnabled) {
		UFont* RenderFont = GEngine->GetLargeFont();
		Canvas->SetDrawColor(FColor::Yellow);

		// draw MPH, RPM and current gear
		float X, Y;
		Canvas->GetCenter(X, Y);
		float YLine = Y * 2.f - 50.f;
		float Scaling = 2.f;
		Canvas->DrawText(RenderFont, FString::Printf(TEXT("%d mph"), (int)ForwardSpeedMPH), X - 100, YLine, Scaling, Scaling);
		Canvas->DrawText(RenderFont, FString::Printf(TEXT("[%d]"), (int)PVehicleOutput->CurrentGear), X, YLine, Scaling, Scaling);
		Canvas->DrawText(RenderFont, FString::Printf(TEXT("%d rpm"), (int)PVehicleOutput->EngineRPM), X + 50, YLine, Scaling, Scaling);

		FVector2D DialPos(X + 10, YLine - 40);
		float DialRadius = 50;
		DrawDial(Canvas, DialPos, DialRadius, PVehicleOutput->EngineRPM, EngineSetup.MaxRPM);
	}

	UFont* RenderFont = GEngine->GetMediumFont();
	// draw drive data
	{
		Canvas->SetDrawColor(FColor::White);
		YPos += 16;

		if (bMechanicalSimEnabled) {
			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("RPM: %.1f (ChangeUp RPM %.0f, ChangeDown RPM %.0f)"), GetEngineRotationSpeed(), TransmissionSetup.ChangeUpRPM, TransmissionSetup.ChangeDownRPM), 4, YPos);

			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Gear: %d (Target %d)"), GetCurrentGear(), GetTargetGear()), 4, YPos);
		}
		//YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Drag: %.1f"), DebugDragMagnitude), 4, YPos);

		YPos += 16;
		for (int i = 0; i < PVehicleOutput->Wheels.Num(); i++) {
			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("WheelLoad: [%d] %1.f N"), i, Chaos::CmToM(WheelStatus[i].SpringForce)), 4, YPos);
		}

		YPos += 16;
		for (int i = 0; i < PVehicleOutput->Wheels.Num(); i++) {
			if (WheelStatus[i].PhysMaterial.IsValid()) {
				YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("SurfaceFriction: [%d] %.2f"), i, WheelStatus[i].PhysMaterial->Friction), 4, YPos);
			}
		}
	}

	if (DebugPage == EDebugPages::PerformancePage) {
		if (GWheeledVehicleDebugParams.ResetPerformanceMeasurements) {
			GWheeledVehicleDebugParams.ResetPerformanceMeasurements = false;
			PerformanceMeasure.ResetAll();
		}

		PerformanceMeasure.Enable();

		YPos += 16;
		for (int I = 0; I < PerformanceMeasure.GetNumMeasures(); I++) {
			const Chaos::FTimeAndDistanceMeasure& Measure = PerformanceMeasure.GetMeasure(I);

			YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("%s"), *Measure.ToString()), 4, YPos);
		}
	}

	// draw wheel layout
	if (DebugPage == EDebugPages::FrictionPage) {
		FVector2D MaxSize = GetWheelLayoutDimensions();

		// Draw a top down representation of the wheels in position, with the direction forces being shown
		for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx) {
			auto& PWheel = PVehicleOutput->Wheels[WheelIdx];
			//			FVector Forces = PWheel.GetForceFromFriction();
			//
			//			FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];
			//			UChaosVehicleWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();
			//			check(Wheel);
			//			UPhysicalMaterial* ContactMat = Wheel->GetContactSurfaceMaterial();
			//
			//			const FVector WheelOffset = GetWheelRestingPosition(WheelSetup);
			//
			//			float DrawScale = 300;
			//			FVector2D CentreDrawPosition(350, 400);
			//			FVector2D WheelDrawPosition(WheelOffset.Y, -WheelOffset.X);
			//			WheelDrawPosition *= DrawScale;
			//			WheelDrawPosition /= MaxSize.X;
			//			WheelDrawPosition += CentreDrawPosition;
			//
			//			FVector2D WheelDimensions(Wheel->WheelWidth, Wheel->WheelRadius * 2.0f);
			//			FVector2D HalfDimensions = WheelDimensions * 0.5f;
			//			FCanvasBoxItem BoxItem(WheelDrawPosition - HalfDimensions, WheelDimensions);
			//			BoxItem.SetColor(FColor::Green);
			//			Canvas->DrawItem(BoxItem);
			//
			//			float VisualScaling = 0.0001f;
			//			FVector2D Force2D(Forces.Y * VisualScaling, -Forces.X * VisualScaling);
			//			DrawLine2D(Canvas, WheelDrawPosition, WheelDrawPosition + Force2D, FColor::Red);
			//
			//			float SlipAngle = FMath::Abs(PWheel.GetSlipAngle());
			//			float X = FMath::Sin(SlipAngle) * 50.f;
			//			float Y = FMath::Cos(SlipAngle) * 50.f;
			//
			//			int Xpos = WheelDrawPosition.X + 20;
			//			int Ypos = WheelDrawPosition.Y - 75.f;
			//			DrawLine2D(Canvas, WheelDrawPosition, WheelDrawPosition - FVector2D(X, Y), FColor::White);
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Slip Angle : %d %"), (int)RadToDeg(SlipAngle)), Xpos, Ypos);
			//
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("AccelT : %.1f"), PWheel.GetDriveTorque()), Xpos, Ypos);
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("BrakeT : %.1f"), PWheel.GetBrakeTorque()), Xpos, Ypos);
			//
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Omega : %.2f"), PWheel.GetAngularVelocity()), Xpos, Ypos);
			//
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("GroundV : %.1f"), PWheel.GetRoadSpeed()), Xpos, Ypos);
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("WheelV : %.1f"), PWheel.GetWheelGroundSpeed()), Xpos, Ypos);
			////			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Sx : %.2f"), PWheel.GetNormalizedLongitudinalSlip()), Xpos, Ypos);
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Long Ad Limit : %.2f"), PWheel.LongitudinalAdhesiveLimit), Xpos, Ypos);
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Lat Ad Limit : %.2f"), PWheel.LateralAdhesiveLimit), Xpos, Ypos);
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Side Slip : %.2f"), PWheel.SideSlipModifier), Xpos, Ypos);
			//
			//			if (PWheel.AppliedLinearDriveForce > PWheel.LongitudinalAdhesiveLimit)
			//			{
			//				Canvas->SetDrawColor(FColor::Red);
			//			}
			//			else
			//			{
			//				Canvas->SetDrawColor(FColor::Green);
			//			}
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Ap Drive : %.2f"), PWheel.AppliedLinearDriveForce), Xpos, Ypos);
			//
			//			if (PWheel.AppliedLinearBrakeForce > PWheel.LongitudinalAdhesiveLimit)
			//			{
			//				Canvas->SetDrawColor(FColor::Red);
			//			}
			//			else
			//			{
			//				Canvas->SetDrawColor(FColor::Green);
			//			}
			//			Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Ap Brake : %.2f"), PWheel.AppliedLinearBrakeForce), Xpos, Ypos);
			//			Canvas->SetDrawColor(FColor::White);
			//
			//			//if (PWheel.Setup().EngineEnabled)
			//			//{
			//			//	Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("RPM        : %.1f"), PWheel.GetWheelRPM()), Xpos, Ypos);
			//			//	Ypos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Geared RPM : %.1f"), PTransmission.GetEngineRPMFromWheelRPM(PWheel.GetWheelRPM())), Xpos, Ypos);
			//
			//			//}
			//
			//			if (ContactMat)
			//			{
			//				Canvas->DrawText(RenderFont
			//					, FString::Printf(TEXT("Friction %d"), ContactMat->Friction)
			//					, WheelDrawPosition.X, WheelDrawPosition.Y-95.f);
			//			}
		}
	}

	if (DebugPage == EDebugPages::SteeringPage) {
		//FVector2D MaxSize = GetWheelLayoutDimensions();

		//auto& PSteering = PVehicle->GetSteering();

		//FVector2D J1, J2;
		//for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
		//{
		//	FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];
		//	auto& PWheel = PVehicleOutput->Wheels[WheelIdx];
		//	const FVector WheelOffset = GetWheelRestingPosition(WheelSetup);

		//	float Scale = 300.0f / MaxSize.X;
		//	FVector2D CentreDrawPosition(450, 400);
		//	FVector2D WheelDrawPosition(WheelOffset.Y, -WheelOffset.X);
		//	WheelDrawPosition *= Scale;
		//	WheelDrawPosition += CentreDrawPosition;

		//	if (PWheel.Setup().SteeringEnabled)
		//	{
		//		if (WheelOffset.Y > 0)
		//		{
		//			float SteerAngle = DegToRad(PWheel.GetSteeringAngle());
		//			FVector2D Tire = FVector2D(FMath::Sin(SteerAngle), -FMath::Cos(SteerAngle)) * 30.0f;
		//			FVector2D WPt = WheelDrawPosition;
		//			DrawLine2D(Canvas, WPt - Tire, WPt + Tire, FColor::Black, 8);

		//			if (SteeringSetup.SteeringType == ESteeringType::Ackermann)
		//			{
		//				FVector2D C1, P, C2;
		//				PSteering.Ackermann.GetRightHingeLocations(C1, P, C2);
		//				C1.Y = -C1.Y;
		//				P.Y = -P.Y;
		//				C2.Y = -C2.Y;

		//				FVector2D JPt = WheelDrawPosition + (P - C2) * Scale;
		//				FVector2D CPt = WheelDrawPosition + (C1 - C2) * Scale;
		//				DrawLine2D(Canvas, CPt, JPt, FColor::Orange, 3);
		//				DrawLine2D(Canvas, WPt, JPt, FColor::Orange, 3);
		//				J1 = CPt;
		//			}
		//		}
		//		else
		//		{
		//			float SteerAngle = DegToRad(PWheel.GetSteeringAngle());
		//			FVector2D Tire = FVector2D(FMath::Sin(SteerAngle), -FMath::Cos(SteerAngle)) * 30.0f;
		//			FVector2D WPt = WheelDrawPosition;
		//			DrawLine2D(Canvas, WPt - Tire, WPt + Tire, FColor::Black, 8);

		//			if (SteeringSetup.SteeringType == ESteeringType::Ackermann)
		//			{

		//				FVector2D C1, P, C2;
		//				PSteering.Ackermann.GetLeftHingeLocations(C1, P, C2);
		//				C1.Y = -C1.Y;
		//				P.Y = -P.Y;
		//				C2.Y = -C2.Y;

		//				FVector2D JPt = WheelDrawPosition + (P - C2) * Scale;
		//				FVector2D CPt = WheelDrawPosition + (C1 - C2) * Scale;
		//				DrawLine2D(Canvas, CPt, JPt, FColor::Orange, 3);
		//				DrawLine2D(Canvas, WPt, JPt, FColor::Orange, 3);
		//				J2 = CPt;
		//			}
		//		}
		//	}
		//	else
		//	{
		//		FVector2D CPt = WheelDrawPosition;
		//		FVector2D Tire = FVector2D(0.f, 30.0f);
		//		DrawLine2D(Canvas, CPt - Tire, CPt + Tire, FColor::Black, 8);
		//	}

		//	Canvas->DrawText(RenderFont
		//		, FString::Printf(TEXT("Angle %.1f"), PWheel.GetSteeringAngle())
		//		, WheelDrawPosition.X, WheelDrawPosition.Y - 15.f);

		//}
		//DrawLine2D(Canvas, J1, J2, FColor::Red, 3);
	}

	// draw engine torque curve - just putting engine under transmission
	if (DebugPage == EDebugPages::TransmissionPage && bMechanicalSimEnabled) {
		auto& PTransmissionSetup = TransmissionSetup;

		float MaxTorque = EngineSetup.MaxTorque;
		int CurrentRPM = (int)PVehicleOutput->EngineRPM;
		FVector2D CurrentValue(CurrentRPM, PVehicleOutput->EngineTorque);
		int GraphWidth = 200;
		int GraphHeight = 120;
		int GraphXPos = 200;
		int GraphYPos = 400;

		Canvas->DrawDebugGraph(FString("Engine Torque Graph"), CurrentValue.X, CurrentValue.Y, GraphXPos, GraphYPos, GraphWidth, GraphHeight, FVector2D(0, EngineSetup.MaxRPM), FVector2D(MaxTorque, 0));

		FVector2D LastPoint;
		for (float RPM = 0; RPM <= EngineSetup.MaxRPM; RPM += 10.f) {
			float X = RPM / EngineSetup.MaxRPM;
			float Y = EngineSetup.GetTorqueFromRPM(RPM) / MaxTorque;
			//float Y = PEngine.GetTorqueFromRPM(RPM, false) / MaxTorque;
			FVector2D NextPoint(GraphXPos + GraphWidth * X, GraphYPos + GraphHeight - GraphHeight * Y);
			if (RPM > SMALL_NUMBER) {
				DrawLine2D(Canvas, LastPoint, NextPoint, FColor::Cyan);
			}
			LastPoint = NextPoint;
		}

		Canvas->DrawText(RenderFont, FString::Printf(TEXT("RevRate %.1f"), EngineSetup.EngineRevDownRate), GraphXPos, GraphYPos);
	}

	// draw transmission torque curve
	if (DebugPage == EDebugPages::TransmissionPage && bMechanicalSimEnabled) {
		auto& ESetup = EngineSetup;
		auto& TSetup = TransmissionSetup;
		float MaxTorque = ESetup.MaxTorque;
		float MaxGearRatio = TSetup.ForwardGearRatios[0] * TSetup.FinalRatio;
		// 1st gear always has the highest multiplier
		float LongGearRatio = TSetup.ForwardGearRatios[TSetup.ForwardGearRatios.Num() - 1] * TSetup.FinalRatio;
		int GraphWidth = 400;
		int GraphHeight = 240;
		int GraphXPos = 500;
		int GraphYPos = 150;

		{
			float X = PVehicleOutput->TransmissionRPM;
			float Y = PVehicleOutput->TransmissionTorque;

			FVector2D CurrentValue(X, Y);
			Canvas->DrawDebugGraph(FString("Transmission Torque Graph"), CurrentValue.X, CurrentValue.Y, GraphXPos, GraphYPos, GraphWidth, GraphHeight, FVector2D(0, ESetup.MaxRPM / LongGearRatio), FVector2D(MaxTorque * MaxGearRatio, 0));
		}

		FVector2D LastPoint;

		for (int Gear = 1; Gear <= TSetup.ForwardGearRatios.Num(); Gear++) {
			for (int EngineRPM = 0; EngineRPM <= ESetup.MaxRPM; EngineRPM += 10) {
				float RPMOut = EngineRPM / TSetup.GetGearRatio(Gear);

				float X = RPMOut / (ESetup.MaxRPM / LongGearRatio);
				float Y = ESetup.GetTorqueFromRPM(EngineRPM) * TSetup.GetGearRatio(Gear) / (MaxTorque * MaxGearRatio);
				FVector2D NextPoint(GraphXPos + GraphWidth * X, GraphYPos + GraphHeight - GraphHeight * Y);
				if (EngineRPM > 0) {
					DrawLine2D(Canvas, LastPoint, NextPoint, FColor::Cyan);
				}
				LastPoint = NextPoint;
			}
		}
	}

	// for each of the wheel positions, draw the expected suspension movement limits and the current length
	if (DebugPage == EDebugPages::SuspensionPage) {
		FVector2D MaxSize = GetWheelLayoutDimensions();

		for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx) {
			FChaosWheelSetup& WheelSetup = WheelSetups[WheelIdx];
			UChaosVehicleWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();
			check(Wheel);
			UChaosVehicleWheel* VehicleWheel = Wheels[WheelIdx];

			const FVector WheelOffset = GetWheelRestingPosition(WheelSetup);

			float DrawScale = 200;
			FVector2D CentreDrawPosition(500, 350);
			FVector2D WheelDrawPosition(WheelOffset.Y, -WheelOffset.X);
			WheelDrawPosition *= DrawScale;
			WheelDrawPosition /= MaxSize.X;
			WheelDrawPosition += CentreDrawPosition;

			{
				// suspension resting position
				FVector2D Start = WheelDrawPosition + FVector2D(-10.f, 0.f);
				FVector2D End = Start + FVector2D(20.f, 0.f);
				DrawLine2D(Canvas, Start, End, FColor::Yellow, 2.f);
			}

			float Raise = VehicleWheel->SuspensionMaxRaise;
			float Drop = VehicleWheel->SuspensionMaxDrop;
			float Scale = 5.0f;

			{
				// suspension compression limit
				FVector2D Start = WheelDrawPosition + FVector2D(-20.f, -Raise * Scale);
				FVector2D End = Start + FVector2D(40.f, 0.f);
				DrawLine2D(Canvas, Start, End, FColor::White, 2.f);
				Canvas->DrawText(RenderFont, FString::Printf(TEXT("Raise Limit %.1f"), Raise), Start.X, Start.Y - 16);
			}

			{
				// suspension extension limit
				FVector2D Start = WheelDrawPosition + FVector2D(-20.f, Drop * Scale);
				FVector2D End = Start + FVector2D(40.f, 0.f);
				DrawLine2D(Canvas, Start, End, FColor::White, 2.f);
				Canvas->DrawText(RenderFont, FString::Printf(TEXT("Drop Limit %.1f"), Drop), Start.X, Start.Y);
			}

			{
				// current suspension length
				FVector2D Start = WheelDrawPosition;
				FVector2D End = Start - FVector2D(0.f, VehicleWheel->GetSuspensionOffset() * Scale);
				DrawLine2D(Canvas, Start, End, FColor::Green, 4.f);
			}
		}
	}
#endif
}

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)

float UChaosWheeledVehicleMovementComponent::CalcDialAngle(float CurrentValue, float MaxValue) {
	return (CurrentValue / MaxValue) * 3.f / 2.f * PI - (PI * 0.25f);
}

void UChaosWheeledVehicleMovementComponent::FinalizeSimCallbackData(FChaosVehicleManagerAsyncInput& Input) {
	bool bIsPhysicsStateCreated = true;
	if (GetSkeletalMesh()) {
		if (USkeletalMeshComponent* SkelMesh = GetSkeletalMesh()) {
			bIsPhysicsStateCreated = SkelMesh->IsPhysicsStateCreated();
		}
	}

	CurAsyncInput = nullptr;
	CurAsyncOutput = nullptr;
}

void UChaosWheeledVehicleMovementComponent::DrawDial(UCanvas* Canvas, FVector2D Pos, float Radius, float CurrentValue, float MaxValue) {
	float Angle = CalcDialAngle(CurrentValue, MaxValue);
	FVector2D PtEnd(Pos.X - FMath::Cos(Angle) * Radius, Pos.Y - FMath::Sin(Angle) * Radius);
	DrawLine2D(Canvas, Pos, PtEnd, FColor::White, 3.f);

	for (float I = 0; I < MaxValue; I += 1000.0f) {
		Angle = CalcDialAngle(I, MaxValue);
		PtEnd.Set(-FMath::Cos(Angle) * Radius, -FMath::Sin(Angle) * Radius);
		FVector2D PtStart = PtEnd * 0.8f;
		DrawLine2D(Canvas, Pos + PtStart, Pos + PtEnd, FColor::White, 2.f);
	}

	// the last checkmark
	Angle = CalcDialAngle(MaxValue, MaxValue);
	PtEnd.Set(-FMath::Cos(Angle) * Radius, -FMath::Sin(Angle) * Radius);
	FVector2D PtStart = PtEnd * 0.8f;
	DrawLine2D(Canvas, Pos + PtStart, Pos + PtEnd, FColor::Red, 2.f);
}

#endif

void UChaosWheeledVehicleMovementComponent::FillWheelOutputState() {
	if (const FWheeledVehicleAsyncOutput* CurrentOutput = static_cast<FWheeledVehicleAsyncOutput*>(CurAsyncOutput)) {
		if (CurrentOutput->bValid && PVehicleOutput) {
			for (int WheelIdx = 0; WheelIdx < WheelStatus.Num(); WheelIdx++) {
				auto& PWheel = PVehicleOutput->Wheels[WheelIdx];

				FWheelStatus& State = WheelStatus[WheelIdx];

				State.bIsValid = true;
				State.bInContact = PWheel.bBlockingHit;
				State.ContactPoint = PWheel.ImpactPoint;
				State.PhysMaterial = PWheel.PhysMaterial;
				State.NormalizedSuspensionLength = PWheel.NormalizedSuspensionLength;
				State.SpringForce = PWheel.SpringForce;
				State.SlipAngle = PWheel.SlipAngle;
				State.bIsSlipping = PWheel.bIsSlipping;
				State.SlipMagnitude = PWheel.SlipMagnitude;
				State.bIsSkidding = PWheel.bIsSkidding;
				State.SkidMagnitude = PWheel.SkidMagnitude;
				if (State.bIsSkidding) {
					State.SkidNormal = PWheel.SkidNormal;
					//DrawDebugLine(GetWorld()
					//	, State.ContactPoint
					//	, State.ContactPoint + State.SkidNormal
					//	, FColor::Yellow, true, -1.0f, 0, 4);
				}
				else {
					State.SkidNormal = FVector::ZeroVector;
				}
			}
		}
	}
}


void UChaosWheeledVehicleMovementComponent::BreakWheelStatus(const struct FWheelStatus& Status, bool& bInContact, FVector& ContactPoint, UPhysicalMaterial*& PhysMaterial, float& NormalizedSuspensionLength, float& SpringForce, float& SlipAngle, bool& bIsSlipping, float& SlipMagnitude, bool& bIsSkidding, float& SkidMagnitude, FVector& SkidNormal) {
	bInContact = Status.bInContact;
	ContactPoint = Status.ContactPoint;
	PhysMaterial = Status.PhysMaterial.Get();
	NormalizedSuspensionLength = Status.NormalizedSuspensionLength;
	SpringForce = Status.SpringForce;
	SlipAngle = Status.SlipAngle;
	bIsSlipping = Status.bIsSlipping;
	SlipMagnitude = Status.SlipMagnitude;
	bIsSkidding = Status.bIsSkidding;
	SkidMagnitude = Status.SkidMagnitude;
	SkidNormal = Status.SkidNormal;
}

FWheelStatus UChaosWheeledVehicleMovementComponent::MakeWheelStatus(bool bInContact, FVector& ContactPoint, UPhysicalMaterial* PhysMaterial, float NormalizedSuspensionLength, float SpringForce, float SlipAngle, bool bIsSlipping, float SlipMagnitude, bool bIsSkidding, float SkidMagnitude, FVector& SkidNormal) {
	FWheelStatus Status;
	Status.bInContact = bInContact;
	Status.ContactPoint = ContactPoint;
	Status.PhysMaterial = PhysMaterial;
	Status.NormalizedSuspensionLength = NormalizedSuspensionLength;
	Status.SpringForce = SpringForce;
	Status.SlipAngle = SlipAngle;
	Status.bIsSlipping = bIsSlipping;
	Status.SlipMagnitude = SlipMagnitude;
	Status.bIsSkidding = bIsSkidding;
	Status.SkidMagnitude = SkidMagnitude;
	Status.SkidNormal = SkidNormal;

	return Status;
}

void UChaosWheeledVehicleMovementComponent::BreakWheeledSnapshot(const struct FWheeledSnapshotData& Snapshot, FTransform& Transform, FVector& LinearVelocity, FVector& AngularVelocity, int& SelectedGear, float& EngineRPM, TArray<FWheelSnapshot>& WheelSnapshots) {
	Transform = Snapshot.Transform;
	LinearVelocity = Snapshot.LinearVelocity;
	AngularVelocity = Snapshot.AngularVelocity;
	SelectedGear = Snapshot.SelectedGear;
	EngineRPM = Snapshot.EngineRPM;
	WheelSnapshots = Snapshot.WheelSnapshots;
}

FWheeledSnapshotData UChaosWheeledVehicleMovementComponent::MakeWheeledSnapshot(FTransform Transform, FVector LinearVelocity, FVector AngularVelocity, int SelectedGear, float EngineRPM, const TArray<FWheelSnapshot>& WheelSnapshots) {
	FWheeledSnapshotData Snapshot;
	Snapshot.Transform = Transform;
	Snapshot.LinearVelocity = LinearVelocity;
	Snapshot.AngularVelocity = AngularVelocity;
	Snapshot.SelectedGear = SelectedGear;
	Snapshot.EngineRPM = EngineRPM;
	Snapshot.WheelSnapshots = WheelSnapshots;

	return Snapshot;
}


void UChaosWheeledVehicleMovementComponent::BreakWheelSnapshot(const struct FWheelSnapshot& Snapshot, float& SuspensionOffset, float& WheelRotationAngle, float& SteeringAngle, float& WheelRadius, float& WheelAngularVelocity) {
	SuspensionOffset = Snapshot.SuspensionOffset;
	WheelRotationAngle = Snapshot.WheelRotationAngle;
	SteeringAngle = Snapshot.SteeringAngle;
	WheelRadius = Snapshot.WheelRadius;
	WheelAngularVelocity = Snapshot.WheelAngularVelocity;
}

FWheelSnapshot UChaosWheeledVehicleMovementComponent::MakeWheelSnapshot(float SuspensionOffset, float WheelRotationAngle, float SteeringAngle, float WheelRadius, float WheelAngularVelocity) {
	FWheelSnapshot Snapshot;
	Snapshot.SuspensionOffset = SuspensionOffset;
	Snapshot.WheelRotationAngle = WheelRotationAngle;
	Snapshot.SteeringAngle = SteeringAngle;
	Snapshot.WheelRadius = WheelRadius;
	Snapshot.WheelAngularVelocity = WheelAngularVelocity;

	return Snapshot;
}


void UChaosWheeledVehicleMovementComponent::ParallelUpdate(float DeltaSeconds) {
	if (const FWheeledVehicleAsyncOutput* CurrentOutput = static_cast<FWheeledVehicleAsyncOutput*>(CurAsyncOutput)) {
		if (CurrentOutput->bValid && PVehicleOutput) {
			// TODO: It would be nicer to go through CurAsyncOutput rather
			// than copying into the vehicle, think about non-async path
			PVehicleOutput->CurrentGear = CurAsyncOutput->VehicleSimOutput.CurrentGear;
			PVehicleOutput->TargetGear = CurAsyncOutput->VehicleSimOutput.TargetGear;

			// WHEN RUNNING WITH ASYNC ON & FIXED TIMESTEP THEN WE NEED TO INTERPOLATE BETWEEN THE CURRENT AND NEXT OUTPUT RESULTS
			if (const FWheeledVehicleAsyncOutput* NextOutput = static_cast<FWheeledVehicleAsyncOutput*>(NextAsyncOutput)) {
				PVehicleOutput->EngineRPM = FMath::Lerp(CurAsyncOutput->VehicleSimOutput.EngineRPM, NextAsyncOutput->VehicleSimOutput.EngineRPM, OutputInterpAlpha);
				PVehicleOutput->EngineTorque = FMath::Lerp(CurAsyncOutput->VehicleSimOutput.EngineTorque, NextAsyncOutput->VehicleSimOutput.EngineTorque, OutputInterpAlpha);
				PVehicleOutput->TransmissionRPM = FMath::Lerp(CurAsyncOutput->VehicleSimOutput.TransmissionRPM, NextAsyncOutput->VehicleSimOutput.TransmissionRPM, OutputInterpAlpha);
				PVehicleOutput->TransmissionTorque = FMath::Lerp(CurAsyncOutput->VehicleSimOutput.TransmissionTorque, NextAsyncOutput->VehicleSimOutput.TransmissionTorque, OutputInterpAlpha);

				for (int WheelIdx = 0; WheelIdx < CurrentOutput->VehicleSimOutput.Wheels.Num(); WheelIdx++) {
					const FWheelsOutput& Current = CurrentOutput->VehicleSimOutput.Wheels[WheelIdx];
					const FWheelsOutput& Next = NextOutput->VehicleSimOutput.Wheels[WheelIdx];

					PVehicleOutput->Wheels[WheelIdx].InContact = Current.InContact;
					PVehicleOutput->Wheels[WheelIdx].SteeringAngle = FMath::Lerp(Current.SteeringAngle, Next.SteeringAngle, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].WheelRadius = FMath::Lerp(Current.WheelRadius, Next.WheelRadius, OutputInterpAlpha);
					float DeltaAngle = FMath::FindDeltaAngleRadians(Current.AngularPosition, Next.AngularPosition);
					PVehicleOutput->Wheels[WheelIdx].AngularPosition = Current.AngularPosition + DeltaAngle * OutputInterpAlpha;
					PVehicleOutput->Wheels[WheelIdx].AngularVelocity = FMath::Lerp(Current.AngularVelocity, Next.AngularVelocity, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].LateralAdhesiveLimit = FMath::Lerp(Current.LateralAdhesiveLimit, Next.LateralAdhesiveLimit, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].LongitudinalAdhesiveLimit = FMath::Lerp(Current.LongitudinalAdhesiveLimit, Next.LongitudinalAdhesiveLimit, OutputInterpAlpha);

					PVehicleOutput->Wheels[WheelIdx].bIsSlipping = Current.bIsSlipping;
					PVehicleOutput->Wheels[WheelIdx].SlipMagnitude = FMath::Lerp(Current.SlipMagnitude, Next.SlipMagnitude, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].bIsSkidding = Current.bIsSkidding;
					PVehicleOutput->Wheels[WheelIdx].SkidMagnitude = FMath::Lerp(Current.SkidMagnitude, Next.SkidMagnitude, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].SkidNormal = FMath::Lerp(Current.SkidNormal, Next.SkidNormal, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].SlipAngle = FMath::Lerp(Current.SlipAngle, Next.SlipAngle, OutputInterpAlpha);

					PVehicleOutput->Wheels[WheelIdx].SuspensionOffset = FMath::Lerp(Current.SuspensionOffset, Next.SuspensionOffset, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].SpringForce = FMath::Lerp(Current.SpringForce, Next.SpringForce, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].NormalizedSuspensionLength = FMath::Lerp(Current.NormalizedSuspensionLength, Next.NormalizedSuspensionLength, OutputInterpAlpha);

					PVehicleOutput->Wheels[WheelIdx].bBlockingHit = Current.bBlockingHit;
					PVehicleOutput->Wheels[WheelIdx].ImpactPoint = FMath::Lerp(Current.ImpactPoint, Next.ImpactPoint, OutputInterpAlpha);
					PVehicleOutput->Wheels[WheelIdx].PhysMaterial = Current.PhysMaterial;
				}
			}
			else
			// WHEN ASYNC IS OFF IT STILL GENERATES THE ASYNC CALLBACK BUT THERE IS ONLY EVER THE CURRENT AND NO NEXT OUTPUT TO INTERPOLATE BETWEEN
			{
				PVehicleOutput->EngineRPM = CurAsyncOutput->VehicleSimOutput.EngineRPM;
				PVehicleOutput->EngineTorque = CurAsyncOutput->VehicleSimOutput.EngineTorque;
				PVehicleOutput->TransmissionRPM = CurAsyncOutput->VehicleSimOutput.TransmissionRPM;
				PVehicleOutput->TransmissionTorque = CurAsyncOutput->VehicleSimOutput.TransmissionTorque;

				for (int WheelIdx = 0; WheelIdx < CurrentOutput->VehicleSimOutput.Wheels.Num(); WheelIdx++) {
					const FWheelsOutput& Current = CurrentOutput->VehicleSimOutput.Wheels[WheelIdx];

					PVehicleOutput->Wheels[WheelIdx].InContact = Current.InContact;
					PVehicleOutput->Wheels[WheelIdx].SteeringAngle = Current.SteeringAngle;
					PVehicleOutput->Wheels[WheelIdx].WheelRadius = Current.WheelRadius;
					PVehicleOutput->Wheels[WheelIdx].AngularPosition = Current.AngularPosition;
					PVehicleOutput->Wheels[WheelIdx].AngularVelocity = Current.AngularVelocity;
					PVehicleOutput->Wheels[WheelIdx].LateralAdhesiveLimit = Current.LateralAdhesiveLimit;
					PVehicleOutput->Wheels[WheelIdx].LongitudinalAdhesiveLimit = Current.LongitudinalAdhesiveLimit;

					PVehicleOutput->Wheels[WheelIdx].bIsSlipping = Current.bIsSlipping;
					PVehicleOutput->Wheels[WheelIdx].SlipMagnitude = Current.SlipMagnitude;
					PVehicleOutput->Wheels[WheelIdx].bIsSkidding = Current.bIsSkidding;
					PVehicleOutput->Wheels[WheelIdx].SkidMagnitude = Current.SkidMagnitude;
					PVehicleOutput->Wheels[WheelIdx].SkidNormal = Current.SkidNormal;
					PVehicleOutput->Wheels[WheelIdx].SlipAngle = Current.SlipAngle;

					PVehicleOutput->Wheels[WheelIdx].SuspensionOffset = Current.SuspensionOffset;
					PVehicleOutput->Wheels[WheelIdx].SpringForce = Current.SpringForce;
					PVehicleOutput->Wheels[WheelIdx].NormalizedSuspensionLength = Current.NormalizedSuspensionLength;

					PVehicleOutput->Wheels[WheelIdx].bBlockingHit = Current.bBlockingHit;
					PVehicleOutput->Wheels[WheelIdx].ImpactPoint = Current.ImpactPoint;
					PVehicleOutput->Wheels[WheelIdx].PhysMaterial = Current.PhysMaterial;
				}
			}
		}
	}

	FillWheelOutputState(); // exposes wheel/suspension data to blueprint
}

void UChaosWheeledVehicleMovementComponent::SetWheelClass(int WheelIndex, TSubclassOf<UChaosVehicleWheel> InWheelClass) {
	if (UpdatedPrimitive && InWheelClass) {
		FBodyInstance* TargetInstance = GetBodyInstance();

		if (TargetInstance && WheelIndex < Wheels.Num()) {
			FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
				if (VehicleSimulationPT) {
					UChaosVehicleWheel* OldWheel = Wheels[WheelIndex];
					UChaosVehicleWheel* NewWheel = NewObject<UChaosVehicleWheel>(this, InWheelClass);
					NewWheel->Init(this, WheelIndex);

					VehicleSimulationPT->InitializeWheel(WheelIndex, &NewWheel->GetPhysicsWheelConfig());
					VehicleSimulationPT->InitializeSuspension(WheelIndex, &NewWheel->GetPhysicsSuspensionConfig());

					Wheels[WheelIndex] = NewWheel;

					OldWheel->Shutdown();
				}
			});
		}
	}
}

void UChaosWheeledVehicleSimulation::InitializeWheel(int WheelIndex, const Chaos::FSimpleWheelConfig* InWheelSetup) {
	if (PVehicle->IsValid() && InWheelSetup && WheelIndex < PVehicle->Wheels.Num()) {
		PVehicle->Wheels[WheelIndex].SetupPtr = InWheelSetup;
		PVehicle->Wheels[WheelIndex].SetWheelRadius(InWheelSetup->WheelRadius);
	}
}

void UChaosWheeledVehicleSimulation::InitializeSuspension(int WheelIndex, const Chaos::FSimpleSuspensionConfig* InSuspensionSetup) {
	if (PVehicle->IsValid() && InSuspensionSetup && WheelIndex < PVehicle->Suspension.Num()) {
		PVehicle->Suspension[WheelIndex].SetupPtr = InSuspensionSetup;
	}
}

FWheeledSnapshotData UChaosWheeledVehicleMovementComponent::GetSnapshot() const {
	FWheeledSnapshotData WheelSnapshotData;
	UChaosVehicleMovementComponent::GetBaseSnapshot(WheelSnapshotData);

	WheelSnapshotData.EngineRPM = PVehicleOutput->EngineRPM;
	WheelSnapshotData.SelectedGear = PVehicleOutput->CurrentGear;
	WheelSnapshotData.WheelSnapshots.SetNum(Wheels.Num());

	int WheelIdx = 0;
	for (const UChaosVehicleWheel* Wheel : Wheels) {
		WheelSnapshotData.WheelSnapshots[WheelIdx].SteeringAngle = Wheel->GetSteerAngle();
		WheelSnapshotData.WheelSnapshots[WheelIdx].SuspensionOffset = Wheel->GetSuspensionOffset();
		WheelSnapshotData.WheelSnapshots[WheelIdx].WheelRotationAngle = Wheel->GetRotationAngle();
		WheelSnapshotData.WheelSnapshots[WheelIdx].WheelRadius = Wheel->GetWheelRadius();
		WheelSnapshotData.WheelSnapshots[WheelIdx].WheelAngularVelocity = Wheel->GetWheelAngularVelocity();

		WheelIdx++;
	}

	return WheelSnapshotData;
}

void UChaosWheeledVehicleMovementComponent::SetSnapshot(const FWheeledSnapshotData& SnapshotIn) {
	UChaosVehicleMovementComponent::SetBaseSnapshot(SnapshotIn);

	FBodyInstance* TargetInstance = GetBodyInstance();
	if (TargetInstance) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			const FWheeledSnapshotData* WheelSnapshotData = static_cast<const FWheeledSnapshotData*>(&SnapshotIn);

			ensure(WheelSnapshotData->WheelSnapshots.Num() == VehicleSimulationPT->PVehicle->Wheels.Num());
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle) {
				for (int WheelIdx = 0; WheelIdx < WheelSnapshotData->WheelSnapshots.Num(); WheelIdx++) {
					ensure(VehicleSimulationPT->PVehicle->Wheels.Num() == VehicleSimulationPT->PVehicle->Suspension.Num());

					if (WheelIdx < VehicleSimulationPT->PVehicle->Wheels.Num()) {
						const FWheelSnapshot& Data = WheelSnapshotData->WheelSnapshots[WheelIdx];
						Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIdx];
						Chaos::FSimpleSuspensionSim& VehicleSuspension = VehicleSimulationPT->PVehicle->Suspension[WheelIdx];

						VehicleWheel.SetSteeringAngle(Data.SteeringAngle);
						VehicleSuspension.SetSuspensionLength(Data.SuspensionOffset, Data.WheelRadius);
						VehicleWheel.SetAngularPosition(FMath::DegreesToRadians(-Data.WheelRotationAngle));
						VehicleWheel.SetWheelRadius(Data.WheelRadius);
						VehicleWheel.SetAngularVelocity(Data.WheelAngularVelocity);
					}
				}
			}
		});
	}
}

//////////////////////////////////////////////////////////////////////////

void UChaosWheeledVehicleMovementComponent::SetMaxEngineTorque(float Torque) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle) {
				VehicleSimulationPT->PVehicle->GetEngine().SetMaxTorque(Torque);
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetDragCoefficient(float DragCoeff) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle) {
				VehicleSimulationPT->PVehicle->GetAerodynamics().SetDragCoefficient(DragCoeff);
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetDownforceCoefficient(float DownforceCoeff) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle) {
				VehicleSimulationPT->PVehicle->GetAerodynamics().SetDownforceCoefficient(DownforceCoeff);
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetDifferentialFrontRearSplit(float FrontRearSplit) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle) {
				VehicleSimulationPT->PVehicle->GetDifferential().FrontRearSplit = FrontRearSplit;
			}
		});
	}
}


void UChaosWheeledVehicleMovementComponent::SetTractionControlEnabled(int WheelIndex, bool Enabled) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.TractionControlEnabled = Enabled;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetABSEnabled(int WheelIndex, bool Enabled) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.ABSEnabled = Enabled;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetAffectedByBrake(int WheelIndex, bool Enabled) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.BrakeEnabled = Enabled;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetAffectedByHandbrake(int WheelIndex, bool Enabled) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.HandbrakeEnabled = Enabled; // this is affecting all vehicles - needs to be per instance 
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetAffectedBySteering(int WheelIndex, bool Enabled) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.SteeringEnabled = Enabled;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetAffectedByEngine(int WheelIndex, bool Enabled) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.EngineEnabled = Enabled;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelRadius(int WheelIndex, float Radius) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.SetWheelRadius(Radius);
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelFrictionMultiplier(int WheelIndex, float Friction) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.FrictionMultiplier = Friction;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelSlipGraphMultiplier(int WheelIndex, float Multiplier) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.LateralSlipGraphMultiplier = Multiplier;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelMaxBrakeTorque(int WheelIndex, float Torque) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.MaxBrakeTorque = Torque;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelHandbrakeTorque(int WheelIndex, float Torque) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.HandbrakeTorque = Torque;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelMaxSteerAngle(int WheelIndex, float AngleDegrees) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.MaxSteeringAngle = AngleDegrees;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetTorqueCombineMethod(ETorqueCombineMethod InCombineMethod, int32 WheelIndex) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.SetTorqueCombineMethod(static_cast<Chaos::FSimpleWheelConfig::EExternalTorqueCombineMethod>(InCombineMethod));
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetDriveTorque(float DriveTorque, int32 WheelIndex) {
	SetSleeping(false);

	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.SetDriveTorqueOverride(Chaos::TorqueMToCm(DriveTorque));
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetBrakeTorque(float BrakeTorque, int32 WheelIndex) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.SetBrakeTorqueOverride(Chaos::TorqueMToCm(BrakeTorque));
			}
		});
	}
}


void UChaosWheeledVehicleMovementComponent::SetWheelLoadRatio(int32 WheelIndex, float LoadRatio) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleSuspensionSim& VehicleWheel = VehicleSimulationPT->PVehicle->Suspension[WheelIndex];

				VehicleWheel.AccessSetup().WheelLoadRatio = LoadRatio;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelSlideSlipModifier(int32 WheelIndex, float SlipModifier) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.SideSlipModifier = SlipModifier;
			}
		});
	}
}

void UChaosWheeledVehicleMovementComponent::SetWheelCorneringStiffness(int32 WheelIndex, float CorneringStiff) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.CorneringStiffness = (CorneringStiff * 10000.0f);
			}
		});
	}
}

//void UChaosWheeledVehicleMovementComponent::SetWheelFrictionMultiplier(float Friction, int32 WheelIndex)
//{
//
//	if (FBodyInstance* TargetInstance = GetBodyInstance())
//	{
//		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
//			{
//				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
//				{
//					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];
//
//					VehicleWheel.FrictionMultiplier = Friction;
//				}
//			});
//	}
//
//}

void UChaosWheeledVehicleMovementComponent::SetWheelLateralSlipGraph(int32 WheelIndex, FRuntimeFloatCurve LateralSlipGraph) {
	if (FBodyInstance* TargetInstance = GetBodyInstance()) {
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis) {
			if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num()) {
				Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

				VehicleWheel.AccessSetup().LateralSlipGraph.Empty();

				float NumSamples = 20;
				float MinTime = 0.f, MaxTime = 0.f;
				LateralSlipGraph.GetRichCurveConst()->GetTimeRange(MinTime, MaxTime);


				if (MaxTime > 0.0f) {
					for (float X = 0; X <= MaxTime; X += (MaxTime / NumSamples)) {
						float MinVal = 0.f, MaxVal = 0.f;
						LateralSlipGraph.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
						float Y = LateralSlipGraph.GetRichCurveConst()->Eval(X) * 10000.0f;
						VehicleWheel.AccessSetup().LateralSlipGraph.Add(Chaos::FVec2(X, Y));
					}
				}
			}
		});
	}
}


TUniquePtr<Chaos::FSimpleWheeledVehicle> UChaosWheeledVehicleMovementComponent::CreatePhysicsVehicle() {
	// Make the Vehicle Simulation class that will be updated from the physics thread async callback
	VehicleSimulationPT = new UChaosWheeledVehicleSimulation();
	VehicleSimulationPT->GripSetup = GripSetup;
	VehicleSimulationPT->DriftSetup = DriftSetup;

	PVehicleOutput = MakeUnique<FWheeledVehicleOutput>(); // create physics output container
	return MakeUnique<Chaos::FSimpleWheeledVehicle>(); // create physics sim
}


float UChaosWheeledVehicleMovementComponent::GetSuspensionOffset(int WheelIndex) {
	float Offset = 0.f;

	FChaosWheelSetup& WheelSetup = WheelSetups[WheelIndex];
	if (GetBodyInstance()) {
		FTransform VehicleWorldTransform = GetBodyInstance()->GetUnrealWorldTransform();
		if (UChaosVehicleWheel* Wheel = WheelSetups[WheelIndex].WheelClass.GetDefaultObject()) {
			if (WheelStatus[WheelIndex].bIsValid) {
				if (WheelStatus[WheelIndex].bInContact) {
					FVector LocalPos = GetWheelRestingPosition(WheelSetup);
					FVector LocalHitPoint = VehicleWorldTransform.InverseTransformPosition(WheelStatus[WheelIndex].ContactPoint);
					Offset = LocalHitPoint.Z - LocalPos.Z + PVehicleOutput->Wheels[WheelIndex].WheelRadius;
					Offset = FMath::Clamp(Offset, -Wheel->SuspensionMaxDrop, Wheel->SuspensionMaxRaise);
				}
				else {
					Offset = -Wheel->SuspensionMaxDrop;
				}

				CachedState[WheelIndex].bIsValid = true;
				CachedState[WheelIndex].WheelOffset = Offset;
			}
			else {
				if (VehicleState.bSleeping && CachedState[WheelIndex].bIsValid) {
					Offset = CachedState[WheelIndex].WheelOffset;
				}
				else {
					ECollisionChannel SpringCollisionChannel = ECollisionChannel::ECC_WorldDynamic;
					FCollisionResponseParams ResponseParams;
					ResponseParams.CollisionResponse = WheelTraceCollisionResponses;

					TArray<AActor*> ActorsToIgnore;
					ActorsToIgnore.Add(GetPawnOwner()); // ignore self in scene query

					FCollisionQueryParams TraceParams(NAME_None, FCollisionQueryParams::GetUnknownStatId(), false, nullptr);
					TraceParams.bReturnPhysicalMaterial = true; // we need this to get the surface friction coefficient
					TraceParams.AddIgnoredActors(ActorsToIgnore);
					TraceParams.bTraceComplex = (Wheels[WheelIndex]->SweepType == ESweepType::ComplexSweep);

					FVector LocalDirection = Wheel->SuspensionAxis;
					FVector WorldLocation = VehicleWorldTransform.TransformPosition(GetWheelRestingPosition(WheelSetup));
					FVector WorldDirection = VehicleWorldTransform.TransformVector(LocalDirection);

					FVector TraceStart = WorldLocation - WorldDirection * (Wheel->SuspensionMaxRaise);
					FVector TraceEnd = WorldLocation + WorldDirection * (Wheel->SuspensionMaxDrop + Wheel->WheelRadius);

					FHitResult HitResult;
					GetWorld()->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, SpringCollisionChannel, TraceParams, ResponseParams);

					if (HitResult.bBlockingHit) {
						FVector LocalPos = GetWheelRestingPosition(WheelSetup);
						FVector LocalHitPoint = VehicleWorldTransform.InverseTransformPosition(HitResult.ImpactPoint);
						Offset = LocalHitPoint.Z - LocalPos.Z + Wheel->WheelRadius;
						Offset = FMath::Clamp(Offset, -Wheel->SuspensionMaxDrop, Wheel->SuspensionMaxRaise);
					}
					else {
						Offset = -Wheel->SuspensionMaxDrop;
					}

					CachedState[WheelIndex].bIsValid = true;
					CachedState[WheelIndex].WheelOffset = Offset;
				}
			}
		}
	}

	return Offset;
}


TUniquePtr<FChaosVehicleAsyncInput> UChaosWheeledVehicleMovementComponent::SetCurrentAsyncInputOutput(int32 InputIdx, FChaosVehicleManagerAsyncOutput* CurOutput, FChaosVehicleManagerAsyncOutput* NextOutput, float Alpha, int32 VehicleManagerTimestamp) {
	TUniquePtr<FWheeledVehicleAsyncInput> CurInput = MakeUnique<FWheeledVehicleAsyncInput>();
	SetCurrentAsyncInputOutputInternal(CurInput.Get(), InputIdx, CurOutput, NextOutput, Alpha, VehicleManagerTimestamp);
	return CurInput;
}


/************************************************************************/
/* Setup the current async I/O data                                     */
/************************************************************************/
void UChaosWheeledVehicleMovementComponent::SetCurrentAsyncInputOutputInternal(FChaosVehicleAsyncInput* CurInput, int32 InputIdx, FChaosVehicleManagerAsyncOutput* CurOutput, FChaosVehicleManagerAsyncOutput* NextOutput, float Alpha, int32 VehicleManagerTimestamp) {
	ensure(CurAsyncInput == nullptr); //should be reset after it was filled
	ensure(CurAsyncOutput == nullptr); //should get reset after update is done

	CurAsyncInput = CurInput;
	CurAsyncInput->Vehicle = this;
	CurAsyncType = CurInput->Type;
	NextAsyncOutput = nullptr;
	OutputInterpAlpha = 0.f;

	// We need to find our vehicle in the output given
	if (CurOutput) {
		for (int32 PendingOutputIdx = 0; PendingOutputIdx < OutputsWaitingOn.Num(); ++PendingOutputIdx) {
			// Found the correct pending output, use index to get the vehicle.
			if (OutputsWaitingOn[PendingOutputIdx].Timestamp == CurOutput->Timestamp) {
				const int32 VehicleIdx = OutputsWaitingOn[PendingOutputIdx].Idx;
				FWheeledVehicleAsyncOutput* VehicleOutput = CurOutput->VehicleOutputs[VehicleIdx].Get();
				if (VehicleOutput && VehicleOutput->bValid && VehicleOutput->Type == CurAsyncType) {
					CurAsyncOutput = VehicleOutput;

					if (NextOutput && NextOutput->Timestamp == CurOutput->Timestamp) {
						// This can occur when substepping - in this case, VehicleOutputs will be in the same order in NextOutput and CurOutput.
						FWheeledVehicleAsyncOutput* VehicleNextOutput = NextOutput->VehicleOutputs[VehicleIdx].Get();
						if (VehicleNextOutput && VehicleNextOutput->bValid && VehicleNextOutput->Type == CurAsyncType) {
							NextAsyncOutput = VehicleNextOutput;
							OutputInterpAlpha = Alpha;
						}
					}
				}

				// these are sorted by timestamp, we are using latest, so remove entries that came before it.
				TArray<FAsyncOutputWrapper> NewOutputsWaitingOn;
				for (int32 CopyIndex = PendingOutputIdx; CopyIndex < OutputsWaitingOn.Num(); ++CopyIndex) {
					NewOutputsWaitingOn.Add(OutputsWaitingOn[CopyIndex]);
				}

				OutputsWaitingOn = MoveTemp(NewOutputsWaitingOn);
				break;
			}
		}
	}

	if (NextOutput && CurOutput) {
		if (NextOutput->Timestamp != CurOutput->Timestamp) {
			// NextOutput and CurOutput occurred in different steps, so we need to search for our specific vehicle.
			for (int32 PendingOutputIdx = 0; PendingOutputIdx < OutputsWaitingOn.Num(); ++PendingOutputIdx) {
				// Found the correct pending output, use index to get the vehicle.
				if (OutputsWaitingOn[PendingOutputIdx].Timestamp == NextOutput->Timestamp) {
					FWheeledVehicleAsyncOutput* VehicleOutput = NextOutput->VehicleOutputs[OutputsWaitingOn[PendingOutputIdx].Idx].Get();
					if (VehicleOutput && VehicleOutput->bValid && VehicleOutput->Type == CurAsyncType) {
						NextAsyncOutput = VehicleOutput;
						OutputInterpAlpha = Alpha;
					}
					break;
				}
			}
		}
	}

	FAsyncOutputWrapper& NewOutput = OutputsWaitingOn.AddDefaulted_GetRef();
	NewOutput.Timestamp = VehicleManagerTimestamp;
	NewOutput.Idx = InputIdx;
}


UPhysicalMaterial* UChaosWheeledVehicleMovementComponent::GetPhysMaterial(int WheelIndex) {
	return WheelStatus[WheelIndex].PhysMaterial.Get();
}


FChaosWheelSetup::FChaosWheelSetup() : WheelClass(UChaosVehicleWheel::StaticClass())
                                       //	, SteeringBoneName(NAME_None)
                                       , BoneName(NAME_None), AdditionalOffset(0.0f) {
}

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_ENABLE_OPTIMIZATION
#endif


FWheeledVehicleAsyncInput::FWheeledVehicleAsyncInput() : GravityZ(0.f) {
}


void FWheeledVehicleAsyncInput::ApplyDeferredForces(Chaos::FRigidBodyHandle_Internal* RigidHandle) const {
	check(Vehicle);
	check(Vehicle->VehicleSimulationPT);
	Vehicle->VehicleSimulationPT->ApplyDeferredForces(RigidHandle);
}

/************************************************************************/
/* Async simulation callback on the Physics Thread                      */
/************************************************************************/
TUniquePtr<FWheeledVehicleAsyncOutput> FWheeledVehicleAsyncInput::Simulate(UWorld* World, const float DeltaSeconds, const float TotalSeconds, bool& bWakeOut) const {
	TUniquePtr<FWheeledVehicleAsyncOutput> Output = MakeUnique<FWheeledVehicleAsyncOutput>();

	//UE_LOG(LogChaos, Warning, TEXT("Vehicle Physics Thread Tick %f"), DeltaSeconds);

	//support nullptr because it allows us to go wide on filling the async inputs
	if (Proxy == nullptr) {
		return Output;
	}

	// We now have access to the physics representation of the chassis on the physics thread async tick
	Chaos::FRigidBodyHandle_Internal* Handle = Proxy->GetPhysicsThreadAPI();

	// FILL OUTPUT DATA HERE THAT WILL GET PASSED BACK TO THE GAME THREAD
	Vehicle->VehicleSimulationPT->TickVehicle(World, DeltaSeconds, *this, *Output.Get(), Handle);

	Output->bValid = true;

	return MoveTemp(Output);
}
