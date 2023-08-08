// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AerodynamicsSystem.h"
#include "UObject/ObjectMacros.h"
#include "Templates/SubclassOf.h"
#include "Curves/CurveFloat.h"
#include "GameFramework/PawnMovementComponent.h"
#include "AerofoilSystem.h"
#include "ArcadeSystem.h"
#include "ThrustSystem.h"
#include "PhysicsProxy/SingleParticlePhysicsProxyFwd.h"
#include "SnapshotData.h"
#include "DeferredForces.h"
#include "CollisionQueryParams.h"
#include "EngineSystem.h"
#include "SimpleVehicle.h"
#include "Logging/LogMacros.h"
#include "SteeringSystem.h"
#include "TransmissionSystem.h"

#include "ChaosVehicleMovementComponent.generated.h"


DECLARE_LOG_CATEGORY_EXTERN(LogVehicle, Log, All);
inline DEFINE_LOG_CATEGORY(LogVehicle);


enum VEHICLES_API EDebugPages : uint8
{
	BasicPage = 0,
	PerformancePage,
	SteeringPage,
	FrictionPage,
	SuspensionPage,
	TransmissionPage,

	MaxDebugPages	// keep as last value
};

struct VEHICLES_API FVehicleDebugParams
{
	bool ShowCOM = false;
	bool ShowModelOrigin = false;
	bool ShowAllForces = false;
	bool ShowAerofoilForces = false;
	bool ShowAerofoilSurface = false;
	bool DisableTorqueControl = false;
	bool DisableStabilizeControl = false;
	bool DisableAerodynamics = false;
	bool DisableAerofoils = false;
	bool DisableThrusters = false;
	bool BatchQueries = true;
	bool CacheTraceOverlap = false;
	float ForceDebugScaling = 0.0006f;
	float SleepCounterThreshold = 15;
	bool DisableVehicleSleep = false;
	bool EnableMultithreading = true;
	float SetMaxMPH = 0.0f;
	float ControlInputWakeTolerance = 0.02f;
};


UENUM()
enum class EVehicleDifferential : uint8
{
	Undefined,
	AllWheelDrive,
	FrontWheelDrive,
	RearWheelDrive,
};


inline FVehicleDebugParams GVehicleDebugParams;

class VEHICLES_API UChaosVehicleMovementComponent;


class UCanvas;

struct VEHICLES_API FControlInputs
{
	FControlInputs()
	 : SteeringInput(0.f)
	 , ThrottleInput(0.f)
	 , BrakeInput(0.f)
	 , HandbrakeInput(0.f)
	 , TransmissionType(Chaos::ETransmissionType::Automatic)
	 , GearUpInput(false)
	 , GearDownInput(false)
	{

	}

	// Steering output to physics system. Range -1...1
	float SteeringInput;

	// Accelerator output to physics system. Range 0...1
	float ThrottleInput;

	// Brake output to physics system. Range 0...1
	float BrakeInput;

	// Handbrake output to physics system. Range 0...1
	bool HandbrakeInput;


	Chaos::ETransmissionType TransmissionType;

	bool GearUpInput;
	bool GearDownInput;

	Chaos::FControlInputs GetChaosInputs() const {
		auto input = Chaos::FControlInputs();
		input.Throttle = ThrottleInput;
		input.Brake = BrakeInput;
		input.Steering = SteeringInput;
		return input;
	}
};

struct FBodyInstance;

USTRUCT()
struct VEHICLES_API FVehicleReplicatedState
{
	GENERATED_USTRUCT_BODY()

	FVehicleReplicatedState()
	{
		SteeringInput = 0.f;
		ThrottleInput = 0.f;
		BrakeInput = 0.f;
		HandbrakeInput = 0.f;
		TargetGear = 0;
		ThrottleUp = 0.f;
		ThrottleDown = 0.f;
	}

	// input replication: steering
	UPROPERTY()
	float SteeringInput;

	// input replication: throttle
	UPROPERTY()
	float ThrottleInput;

	// input replication: brake
	UPROPERTY()
	float BrakeInput;

	// input replication: handbrake
	UPROPERTY()
	bool HandbrakeInput;

	// state replication: gear
	UPROPERTY()
	int32 TargetGear;

	// input replication: increase throttle
	UPROPERTY()
	float ThrottleUp;

	// input replication: decrease throttle
	UPROPERTY()
	float ThrottleDown;

};


USTRUCT()
struct VEHICLES_API FVehicleTorqueControlConfig
{
public:
	GENERATED_USTRUCT_BODY()

	FVehicleTorqueControlConfig()
	{
		InitDefaults();
	}

	/** Torque Control Enabled */
	UPROPERTY(EditAnywhere, Category = Setup)
	bool Enabled;

	/** Yaw Torque Scaling */
	UPROPERTY(EditAnywhere, Category = Setup)
	float YawTorqueScaling;

	UPROPERTY(EditAnywhere, Category = Setup)
	float YawFromSteering;
	
	UPROPERTY(EditAnywhere, Category = Setup)
	float YawFromRollTorqueScaling;

	/** Pitch Torque Scaling */
	UPROPERTY(EditAnywhere, Category = Setup)
	float PitchTorqueScaling;

	/** Roll Torque Scaling */
	UPROPERTY(EditAnywhere, Category = Setup)
	float RollTorqueScaling;

	UPROPERTY(EditAnywhere, Category = Setup)
	float RollFromSteering;

	/** Rotation damping */
	UPROPERTY(EditAnywhere, Category = Setup)
	float RotationDamping;

	const Chaos::FTorqueControlConfig& GetTorqueControlConfig()
	{
		FillTorqueControlSetup();
		return PTorqueControlConfig;
	}

	void InitDefaults()
	{
		Enabled = false;
		YawTorqueScaling = 0.0f;
		YawFromSteering = 0.0f;
		YawFromRollTorqueScaling = 0.0f;
		PitchTorqueScaling = 0.0f;
		RollTorqueScaling = 0.0f;
		RollFromSteering = 0.0f;
		RotationDamping = 0.02f;
	}

private:
	void FillTorqueControlSetup()
	{
		PTorqueControlConfig.Enabled = Enabled;
		PTorqueControlConfig.YawTorqueScaling = YawTorqueScaling;
		PTorqueControlConfig.YawFromSteering = YawFromSteering;
		PTorqueControlConfig.YawFromRollTorqueScaling = YawFromRollTorqueScaling;
		PTorqueControlConfig.PitchTorqueScaling = PitchTorqueScaling;
		PTorqueControlConfig.RollTorqueScaling = RollTorqueScaling;
		PTorqueControlConfig.RollFromSteering = RollFromSteering;
		PTorqueControlConfig.RotationDamping = RotationDamping;

	}

	Chaos::FTorqueControlConfig PTorqueControlConfig;
};

USTRUCT()
struct VEHICLES_API FVehicleTargetRotationControlConfig
{
public:
	GENERATED_USTRUCT_BODY()

	FVehicleTargetRotationControlConfig()
	{
		InitDefaults();
	}

	/** Rotation Control Enabled */
	UPROPERTY(EditAnywhere, Category = Setup)
	bool Enabled;

	UPROPERTY(EditAnywhere, Category = Setup)
	bool bRollVsSpeedEnabled;

	UPROPERTY(EditAnywhere, Category = Setup)
	float RollControlScaling;

	UPROPERTY(EditAnywhere, Category = Setup)
	float RollMaxAngle;

	UPROPERTY(EditAnywhere, Category = Setup)
	float PitchControlScaling;

	UPROPERTY(EditAnywhere, Category = Setup)
	float PitchMaxAngle;

	/** Rotation stiffness */
	UPROPERTY(EditAnywhere, Category = Setup)
	float RotationStiffness;

	/** Rotation damping */
	UPROPERTY(EditAnywhere, Category = Setup)
	float RotationDamping;

	/** Rotation mac accel */
	UPROPERTY(EditAnywhere, Category = Setup)
	float MaxAccel;

	UPROPERTY(EditAnywhere, Category = Setup)
	float AutoCentreRollStrength;

	UPROPERTY(EditAnywhere, Category = Setup)
	float AutoCentrePitchStrength;

	UPROPERTY(EditAnywhere, Category = Setup)
	float AutoCentreYawStrength;

	const Chaos::FTargetRotationControlConfig& GetTargetRotationControlConfig()
	{
		FillTargetRotationControlSetup();
		return PTargetRotationControlConfig;
	}

	void InitDefaults()
	{
		Enabled = false;

		bRollVsSpeedEnabled = false;

		RollControlScaling = 0.f;
		RollMaxAngle = 0.f;
		PitchControlScaling = 0.f;
		PitchMaxAngle = 0.f;

		RotationStiffness = 0.f;
		RotationDamping = 0.2;
		MaxAccel = 0.f;

		AutoCentreRollStrength = 0.f;
		AutoCentrePitchStrength = 0.f;
		AutoCentreYawStrength = 0.f;
	}

private:
	void FillTargetRotationControlSetup()
	{
		PTargetRotationControlConfig.Enabled = Enabled;
		PTargetRotationControlConfig.bRollVsSpeedEnabled = bRollVsSpeedEnabled;
		PTargetRotationControlConfig.RollControlScaling = RollControlScaling;
		PTargetRotationControlConfig.RollMaxAngle = RollMaxAngle;
		PTargetRotationControlConfig.PitchControlScaling = PitchControlScaling;
		PTargetRotationControlConfig.PitchMaxAngle = PitchMaxAngle;
		PTargetRotationControlConfig.RotationStiffness = RotationStiffness;
		PTargetRotationControlConfig.RotationDamping = RotationDamping;
		PTargetRotationControlConfig.MaxAccel = MaxAccel;
		PTargetRotationControlConfig.AutoCentreRollStrength = AutoCentreRollStrength;
		PTargetRotationControlConfig.AutoCentrePitchStrength = AutoCentrePitchStrength;
		PTargetRotationControlConfig.AutoCentreYawStrength = AutoCentreYawStrength;
	}

	Chaos::FTargetRotationControlConfig PTargetRotationControlConfig;
};

USTRUCT()
struct VEHICLES_API FVehicleStabilizeControlConfig
{
public:
	GENERATED_USTRUCT_BODY()

	FVehicleStabilizeControlConfig()
	{
		InitDefaults();
	}

	/** Torque Control Enabled */
	UPROPERTY(EditAnywhere, Category = Setup)
	bool Enabled;

	/** Yaw Torque Scaling */
	UPROPERTY(EditAnywhere, Category = Setup)
	float AltitudeHoldZ;

	UPROPERTY(EditAnywhere, Category = Setup)
	float PositionHoldXY;

	const Chaos::FStabilizeControlConfig& GetStabilizeControlConfig()
	{
		FillStabilizeControlSetup();
		return PStabilizeControlConfig;
	}

	void InitDefaults()
	{
		Enabled = false;
		AltitudeHoldZ = 4.0f;
		PositionHoldXY = 8.0f;
	}

private:
	void FillStabilizeControlSetup()
	{
		PStabilizeControlConfig.Enabled = this->Enabled;
		PStabilizeControlConfig.AltitudeHoldZ = this->AltitudeHoldZ;
		PStabilizeControlConfig.PositionHoldXY = this->PositionHoldXY;
	}

	Chaos::FStabilizeControlConfig PStabilizeControlConfig;

};

/** Input Options */
UENUM()
enum class EInputFunctionType : uint8
{
	LinearFunction = 0,
	SquaredFunction,
	CustomCurve
};

USTRUCT()
struct VEHICLES_API FVehicleInputRateConfig
{
	GENERATED_USTRUCT_BODY()

	/** 
	 * Rate at which the input value rises
	 */
	UPROPERTY(EditAnywhere, Category=VehicleInputRate)
	float RiseRate;

	/**
	 * Rate at which the input value falls
	 */
	UPROPERTY(EditAnywhere, Category=VehicleInputRate)
	float FallRate;

	/**
	 * Controller input curve, various predefined options, linear, squared, or user can specify a custom curve function
	 */
	UPROPERTY(EditAnywhere, Category=VehicleInputRate)
	EInputFunctionType InputCurveFunction;

	/**
	 * Controller input curve - should be a normalized float curve, i.e. time from 0 to 1 and values between 0 and 1
	 * This curve is only sued if the InputCurveFunction above is set to CustomCurve
	 */
	UPROPERTY(EditAnywhere, Category = VehicleInputRate)
	FRuntimeFloatCurve UserCurve;

	FVehicleInputRateConfig() : RiseRate(5.0f), FallRate(5.0f), InputCurveFunction(EInputFunctionType::LinearFunction) { }

	/** Change an output value using max rise and fall rates */
	float InterpInputValue( float DeltaTime, float CurrentValue, float NewValue ) const
	{
		const float DeltaValue = NewValue - CurrentValue;

		// We are "rising" when DeltaValue has the same sign as CurrentValue (i.e. delta causes an absolute magnitude gain)
		// OR we were at 0 before, and our delta is no longer 0.
		const bool bRising = (( DeltaValue > 0.0f ) == ( CurrentValue > 0.0f )) ||
								(( DeltaValue != 0.f ) && ( CurrentValue == 0.f ));

		const float MaxDeltaValue = DeltaTime * ( bRising ? RiseRate : FallRate );
		const float ClampedDeltaValue = FMath::Clamp( DeltaValue, -MaxDeltaValue, MaxDeltaValue );
		return CurrentValue + ClampedDeltaValue;
	}

	float CalcControlFunction(float InputValue)
	{
		// user defined curve

		// else use option from drop down list
		switch (InputCurveFunction)
		{
		case EInputFunctionType::CustomCurve:
		{
			if (UserCurve.GetRichCurveConst() && !UserCurve.GetRichCurveConst()->IsEmpty())
			{
				float Output = FMath::Clamp(UserCurve.GetRichCurveConst()->Eval(FMath::Abs(InputValue)), 0.0f, 1.0f);
				return (InputValue < 0.f)? -Output : Output;
			}
			else
			{
				return InputValue;
			}
		}
		break;
		case EInputFunctionType::SquaredFunction:
		{
			return (InputValue < 0.f) ? -InputValue * InputValue : InputValue * InputValue;
		}
		break;

		case EInputFunctionType::LinearFunction:
		default:
		{
			return InputValue;
		}
		break;
		
		}

	}
};


UENUM()
enum class EVehicleAerofoilType : uint8
{
	Fixed = 0,
	Wing,			// affected by Roll input
	Rudder,			// affected by steering/yaw input
	Elevator		// affected by Pitch input
};


UENUM()
enum class EVehicleThrustType : uint8
{
	Fixed = 0,
	Wing,				// affected by Roll input
	Rudder,				// affected by steering/yaw input
	Elevator,			// affected by Pitch input
//	HelicopterRotor,	// affected by pitch/roll inputs
};


USTRUCT()
struct VEHICLES_API FVehicleDifferentialConfig
{
	GENERATED_USTRUCT_BODY()

	FVehicleDifferentialConfig()
	{
		InitDefaults();
	}

	/** Type of differential */
	UPROPERTY(EditAnywhere, Category=Setup)
	EVehicleDifferential DifferentialType;
	
	/** Ratio of torque split between front and rear (<0.5 means more to front, >0.5 means more to rear, works only with 4W type) */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0"))
	float FrontRearSplit;

	const Chaos::FSimpleDifferentialConfig& GetPhysicsDifferentialConfig()
	{
		FillDifferentialSetup();
		return PDifferentialConfig;
	}

	void InitDefaults()
	{
		DifferentialType = EVehicleDifferential::RearWheelDrive;
		FrontRearSplit = 0.5f;
	}

	void FillDifferentialSetup()
	{
		PDifferentialConfig.DifferentialType = static_cast<Chaos::EDifferentialType>(this->DifferentialType);
		PDifferentialConfig.FrontRearSplit = this->FrontRearSplit;
	}

	Chaos::FSimpleDifferentialConfig PDifferentialConfig;

};
USTRUCT()
struct VEHICLES_API FVehicleEngineConfig
{
	GENERATED_USTRUCT_BODY()

	FVehicleEngineConfig()
	{
		InitDefaults();
	}

	/** Torque [Normalized 0..1] for a given RPM */
	UPROPERTY(EditAnywhere, Category = Setup)
	FRuntimeFloatCurve TorqueCurve;

	/** Max Engine Torque (Nm) is multiplied by TorqueCurve */
	UPROPERTY(EditAnywhere, Category = Setup)
	float MaxTorque;

	/** Maximum revolutions per minute of the engine */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float MaxRPM;

	/** Idle RMP of engine then in neutral/stationary */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float EngineIdleRPM;

	/** Braking effect from engine, when throttle released */
	UPROPERTY(EditAnywhere, Category = Setup)
	float EngineBrakeEffect;

	/** Affects how fast the engine RPM speed up */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float EngineRevUpMOI;

	/** Affects how fast the engine RPM slows down */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float EngineRevDownRate;

	const Chaos::FSimpleEngineConfig& GetPhysicsEngineConfig()
	{
		FillEngineSetup();
		return PEngineConfig;
	}

	void InitDefaults()
	{
		MaxTorque = 300.0f;
		MaxRPM = 4500.0f;
		EngineIdleRPM = 1200.0f;
		EngineBrakeEffect = 0.05f;
		EngineRevUpMOI = 5.0f;
		EngineRevDownRate = 600.0f;
	}

	float GetTorqueFromRPM(float EngineRPM)
	{
		// The source curve does not need to be normalized, however we are normalizing it when it is passed on,
		// since it's the MaxRPM and MaxTorque values that determine the range of RPM and Torque
		float MinVal = 0.f, MaxVal = 0.f;
		this->TorqueCurve.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
		return TorqueCurve.GetRichCurve()->Eval(EngineRPM) / MaxVal * MaxTorque;
	}
private:

	void FillEngineSetup()
	{
		// The source curve does not need to be normalized, however we are normalizing it when it is passed on,
		// since it's the MaxRPM and MaxTorque values that determine the range of RPM and Torque
		PEngineConfig.TorqueCurve.Empty();
		float NumSamples = 20;
		for (float X = 0; X <= this->MaxRPM; X+= (this->MaxRPM / NumSamples))
		{ 
			float MinVal = 0.f, MaxVal = 0.f;
			this->TorqueCurve.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
			float Y = this->TorqueCurve.GetRichCurveConst()->Eval(X) / MaxVal;
			PEngineConfig.TorqueCurve.AddNormalized(Y);
		}
		PEngineConfig.MaxTorque = this->MaxTorque;
		PEngineConfig.MaxRPM = this->MaxRPM;
		PEngineConfig.EngineIdleRPM = this->EngineIdleRPM;
		PEngineConfig.EngineBrakeEffect = this->EngineBrakeEffect;
		PEngineConfig.EngineRevUpMOI = this->EngineRevUpMOI;
		PEngineConfig.EngineRevDownRate = this->EngineRevDownRate;
	}

	Chaos::FSimpleEngineConfig PEngineConfig;

};
USTRUCT()
struct VEHICLES_API FVehicleTransmissionConfig
{
	GENERATED_USTRUCT_BODY()

	FVehicleTransmissionConfig()
	{
		InitDefaults();
	}

	friend class UChaosVehicleElevator;

	/** Whether to use automatic transmission */
	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta=(DisplayName = "Automatic Transmission"))
	bool bUseAutomaticGears;

	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta = (DisplayName = "Automatic Reverse"))
	bool bUseAutoReverse;

	/** The final gear ratio multiplies the transmission gear ratios.*/
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float FinalRatio;

	/** Forward gear ratios */
	UPROPERTY(EditAnywhere, Category = Setup, AdvancedDisplay)
	TArray<float> ForwardGearRatios;

	/** Reverse gear ratio(s) */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	TArray<float> ReverseGearRatios;

	/** Engine Revs at which gear up change ocurrs */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "50000.0", UIMax = "50000.0"), Category = Setup)
	float ChangeUpRPM;

	/** Engine Revs at which gear down change ocurrs */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "50000.0", UIMax = "50000.0"), Category = Setup)
	float ChangeDownRPM;

	/** Time it takes to switch gears (seconds) */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float GearChangeTime;

	/** Mechanical frictional losses mean transmission might operate at 0.94 (94% efficiency) */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float TransmissionEfficiency;

	const Chaos::FSimpleTransmissionConfig& GetPhysicsTransmissionConfig()
	{
		FillTransmissionSetup();
		return PTransmissionConfig;
	}

	void InitDefaults()
	{
		bUseAutomaticGears = true;
		bUseAutoReverse = true;
		FinalRatio = 3.08f;

		ForwardGearRatios.Add(2.85f);
		ForwardGearRatios.Add(2.02f);
		ForwardGearRatios.Add(1.35f);
		ForwardGearRatios.Add(1.0f);

		ReverseGearRatios.Add(2.86f);

		ChangeUpRPM = 4500.0f;
		ChangeDownRPM = 2000.0f;
		GearChangeTime = 0.4f;

		TransmissionEfficiency = 0.9f;
	}

	float GetGearRatio(int32 InGear)
	{
		if (InGear > 0) // a forwards gear
		{
			return ForwardGearRatios[InGear - 1] * FinalRatio;
		}
		else if (InGear < 0) // a reverse gear
		{
			return -ReverseGearRatios[FMath::Abs(InGear) - 1] * FinalRatio;
		}
		else
		{
			return 0.f; // neutral has no ratio
		}
	}


private:

	void FillTransmissionSetup()
	{
		PTransmissionConfig.TransmissionType = this->bUseAutomaticGears ? Chaos::ETransmissionType::Automatic : Chaos::ETransmissionType::Manual;
		PTransmissionConfig.AutoReverse = this->bUseAutoReverse;
		PTransmissionConfig.ChangeUpRPM = this->ChangeUpRPM;
		PTransmissionConfig.ChangeDownRPM = this->ChangeDownRPM;
		PTransmissionConfig.GearChangeTime = this->GearChangeTime;
		PTransmissionConfig.FinalDriveRatio = this->FinalRatio;
		PTransmissionConfig.ForwardRatios.Reset();
		PTransmissionConfig.TransmissionEfficiency = this->TransmissionEfficiency;
		for (float Ratio : this->ForwardGearRatios)
		{
			PTransmissionConfig.ForwardRatios.Add(Ratio);
		}

		PTransmissionConfig.ReverseRatios.Reset();
		for (float Ratio : this->ReverseGearRatios)
		{
			PTransmissionConfig.ReverseRatios.Add(Ratio);
		}
	}

	Chaos::FSimpleTransmissionConfig PTransmissionConfig;

};
UENUM()
enum class ESteeringType : uint8
{
	SingleAngle,
	AngleRatio,
	Ackermann,
};
USTRUCT()
struct VEHICLES_API FVehicleSteeringConfig
{
	GENERATED_USTRUCT_BODY()

	FVehicleSteeringConfig()
	{
		InitDefaults();
	}

	/** Single angle : both wheels steer by the same amount
	 *  AngleRatio   : outer wheels on corner steer less than the inner ones by set ratio 
	 *  Ackermann	 : Ackermann steering principle is applied */
	UPROPERTY(EditAnywhere, Category = SteeringSetup)
	ESteeringType SteeringType;

	/** Only applies when AngleRatio is selected */
	UPROPERTY(EditAnywhere, Category = SteeringSetup)
	float AngleRatio; 

	/** Maximum steering versus forward speed (MPH) */
	UPROPERTY(EditAnywhere, Category = SteeringSetup)
	FRuntimeFloatCurve SteeringCurve;


	const Chaos::FSimpleSteeringConfig& GetPhysicsSteeringConfig(FVector2D WheelTrackDimensions)
	{
		FillSteeringSetup(WheelTrackDimensions);
		return PSteeringConfig;
	}

	void InitDefaults()
	{
		SteeringType = ESteeringType::AngleRatio;
		AngleRatio = 0.7f;

		// Init steering speed curve
		FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
		SteeringCurveData->AddKey(0.f, 1.f);
		SteeringCurveData->AddKey(20.f, 0.8f);
		SteeringCurveData->AddKey(60.f, 0.4f);
		SteeringCurveData->AddKey(120.f, 0.3f);
	}

private:

	void FillSteeringSetup(FVector2D WheelTrackDimensions)
	{

		PSteeringConfig.SteeringType = (Chaos::ESteerType)this->SteeringType;
		PSteeringConfig.AngleRatio = AngleRatio;

		float MinValue = 0.f, MaxValue = 1.f;
		this->SteeringCurve.GetRichCurveConst()->GetValueRange(MinValue, MaxValue);
		float MaxX = this->SteeringCurve.GetRichCurveConst()->GetLastKey().Time;
		PSteeringConfig.SpeedVsSteeringCurve.Empty();
		float NumSamples = 20;
		for (float X = 0; X <= MaxX; X += (MaxX / NumSamples))
		{
			float Y = this->SteeringCurve.GetRichCurveConst()->Eval(X) / MaxValue;
			PSteeringConfig.SpeedVsSteeringCurve.Add(FVector2D(X, Y));
		}

		PSteeringConfig.TrackWidth = WheelTrackDimensions.Y;
		PSteeringConfig.WheelBase = WheelTrackDimensions.X;
	}

	Chaos::FSimpleSteeringConfig PSteeringConfig;

};

USTRUCT()
struct VEHICLES_API FVehicleAerofoilConfig
{
	GENERATED_USTRUCT_BODY()

	FVehicleAerofoilConfig()
	{
		InitDefaults();
	}

	// Does this aerofoil represent a fixed spoiler, an aircraft wing, etc how is controlled.
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	EVehicleAerofoilType AerofoilType;

	// Bone name on mesh where aerofoil is centered
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	FName BoneName;

	// Additional offset to give the aerofoil.
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	FVector Offset;

	// Up Axis of aerofoil.
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	FVector UpAxis;

	// Area of aerofoil surface [Meters Squared] - larger value creates more lift but also more drag
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	float Area;

	// camber of wing - leave as zero for a rudder - can be used to trim/level elevator for level flight
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	float Camber;

	// The angle in degrees through which the control surface moves - leave at 0 if it is a fixed surface
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	float MaxControlAngle;

	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	float StallAngle;

	// cheat to control amount of lift independently from lift
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	float LiftMultiplier;

	// cheat to control amount of drag independently from lift, a value of zero will offer no drag
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	float DragMultiplier;

	const Chaos::FAerofoilConfig& GetPhysicsAerofoilConfig(const UChaosVehicleMovementComponent& MovementComponent)
	{
		FillAerofoilSetup(MovementComponent);
		return PAerofoilConfig;
	}

	void InitDefaults()
	{
		AerofoilType = EVehicleAerofoilType::Fixed;
		BoneName = NAME_None;
		Offset = FVector::ZeroVector;
		UpAxis = FVector(0.f, 0.f, -1.f);
		Area = 1.f;
		Camber = 3.f;
		MaxControlAngle = 0.f;
		StallAngle = 16.f;
		LiftMultiplier = 1.0f;
		DragMultiplier = 1.0f;
	}

private:

	void FillAerofoilSetup(const UChaosVehicleMovementComponent& MovementComponent);

	Chaos::FAerofoilConfig PAerofoilConfig;
};

USTRUCT()
struct VEHICLES_API FVehicleThrustConfig
{
	GENERATED_USTRUCT_BODY()

	FVehicleThrustConfig()
	{
		InitDefaults();
	}

	// Does this aerofoil represent a fixed spoiler, an aircraft wing, etc how is controlled.
	UPROPERTY(EditAnywhere, Category = ThrustSetup)
	EVehicleThrustType ThrustType;

	/** Bone name on mesh where thrust is located */
	UPROPERTY(EditAnywhere, Category = ThrustSetup)
	FName BoneName;

	/** Additional offset to give the location, or use in preference to the bone */
	UPROPERTY(EditAnywhere, Category = ThrustSetup)
	FVector Offset;

	/** Up Axis of thrust. */
	UPROPERTY(EditAnywhere, Category = ThrustSetup)
	FVector ThrustAxis;

	///** How the thrust is applied as the speed increases */
	//UPROPERTY(EditAnywhere, Category = ThrustSetup)
	//FRuntimeFloatCurve ThrustCurve;

	///** Maximum speed after which the thrust will cut off */
	//UPROPERTY(EditAnywhere, Category = ThrustSetup)
	//float MaxSpeed;

	/** Maximum thrust force */
	UPROPERTY(EditAnywhere, Category = ThrustSetup)
	float MaxThrustForce;

	/** The angle in degrees through which the control surface moves - leave at 0 if it is a fixed surface */
	UPROPERTY(EditAnywhere, Category = ThrustSetup)
	float MaxControlAngle;

	// #todo:ControlAxes - X, Y, Z, or X & Y, etc
	const Chaos::FSimpleThrustConfig& GetPhysicsThrusterConfig(const UChaosVehicleMovementComponent& MovementComponent)
	{
		FillThrusterSetup(MovementComponent);
		return PThrusterConfig;
	}

	void InitDefaults()
	{
		ThrustType = EVehicleThrustType::Fixed;
		BoneName = NAME_None;
		Offset = FVector::ZeroVector;
		ThrustAxis = FVector(1,0,0);
		//ThrustCurve.GetRichCurve()->AddKey(0.f, 1.f);
		//ThrustCurve.GetRichCurve()->AddKey(1.f, 1.f);
		MaxThrustForce = 1000.0f;
		MaxControlAngle = 0.f;
	}

private:
	void FillThrusterSetup(const UChaosVehicleMovementComponent &MovementComponent);

	Chaos::FSimpleThrustConfig PThrusterConfig;

};

class VEHICLES_API UChaosVehicleSimulation
{
public:
	virtual ~UChaosVehicleSimulation()
	{
	}

	virtual void UpdateConstraintHandles(TArray<FPhysicsConstraintHandle>& ConstraintHandlesIn) {}

	virtual void ApplyDeferredForces(Chaos::FRigidBodyHandle_Internal* Handle);

	/** Are enough vehicle systems specified such that physics vehicle simulation is possible */
	virtual bool CanSimulate() const { return true; }

	/** Pass control Input to the vehicle systems */
	virtual void ApplyInput(const FControlInputs& ControlInputs, float DeltaTime);

	/** Apply aerodynamic forces to vehicle body */
	virtual void ApplyAerodynamics(float DeltaTime);

	/** Apply Aerofoil forces to vehicle body */
	virtual void ApplyAerofoilForces(float DeltaTime);

	/** Apply Thruster forces to vehicle body */
	virtual void ApplyThrustForces(float DeltaTime, const FControlInputs& ControlInputs);

	/** Apply direct control over vehicle body rotation */
	virtual void ApplyTorqueControl(float DeltaTime, const FControlInputs& InputData);


	/** Add a force to this vehicle */
	void AddForce(const FVector& Force, bool bAllowSubstepping = true, bool bAccelChange = false);
	/** Add a force at a particular position (world space when bIsLocalForce = false, body space otherwise) */
	void AddForceAtPosition(const FVector& Force, const FVector& Position, bool bAllowSubstepping = true, bool bIsLocalForce = false);
	/** Add an impulse to this vehicle */
	void AddImpulse(const FVector& Impulse, bool bVelChange);
	/** Add an impulse to this vehicle and a particular world position */
	void AddImpulseAtPosition(const FVector& Impulse, const FVector& Position);
	/** Add a torque to this vehicle */
	void AddTorqueInRadians(const FVector& Torque, bool bAllowSubstepping = true, bool bAccelChange = false);

	/** Draw debug text for the wheels and suspension */
	virtual void DrawDebug3D();
	UWorld* World;

	// Physics Thread Representation of chassis rigid body
	Chaos::FRigidBodyHandle_Internal* RigidHandle;

	FVehicleState VehicleState;

	// #todo: this isn't very configurable

	FDeferredForces DeferredForces;
public:
	virtual TObjectPtr<Chaos::IVehicleInterface> GetPVehicle() {return nullptr;}

};


/**
 * Base component to handle the vehicle simulation for an actor.
 */
UCLASS(Abstract, hidecategories=(PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class VEHICLES_API UChaosVehicleMovementComponent : public UPawnMovementComponent
{
	friend struct FWheeledVehicleAsyncInput;
	friend class FChaosVehicleManager;

	GENERATED_UCLASS_BODY()

//#todo: these 2 oddities seem out of place

	/** If true, the brake and reverse controls will behave in a more arcade fashion where holding reverse also functions as brake. For a more realistic approach turn this off*/
	UPROPERTY(EditAnywhere, Category = VehicleSetup)
	uint8 bReverseAsBrake : 1;

public:
	/** Mass to set the vehicle chassis to. It's much easier to tweak vehicle settings when
	 * the mass doesn't change due to tweaks with the physics asset. [kg] */
	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float Mass;

	/**
	 * Enable to override the calculated COM position with your own fixed value - this prevents the vehicle handling changing when the asset changes
	 */
	UPROPERTY(EditAnywhere, Category = VehicleSetup)
	bool bEnableCenterOfMassOverride;

	/**
	 * The center of mass override value, this value overrides the calculated COM and the COM offset value in the mesh is also ignored.
	 */
	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta = (EditCondition = "bEnableCenterOfMassOverride"))
	FVector CenterOfMassOverride;

	/** Chassis width used for drag force computation (cm)*/
	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float ChassisWidth;

	/** Chassis height used for drag force computation (cm)*/
	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float ChassisHeight;

	/** DragCoefficient of the vehicle chassis - force resisting forward motion at speed */
	UPROPERTY(EditAnywhere, Category = VehicleSetup)
	float DragCoefficient;

	/** DownforceCoefficient of the vehicle chassis - force pressing vehicle into ground at speed */
	UPROPERTY(EditAnywhere, Category = VehicleSetup)
	float DownforceCoefficient;

	// Drag area in Meters^2
	UPROPERTY(transient)
	float DragArea;

	// Debug drag magnitude last applied
	UPROPERTY(transient)
	float DebugDragMagnitude;

	/** display next debug page */
	static void NextDebugPage();

	/** display previous debug page */
	static void PrevDebugPage();
	/** Scales the vehicle's inertia in each direction (forward, right, up) */
	UPROPERTY(EditAnywhere, Category=VehicleSetup, AdvancedDisplay)
	FVector InertiaTensorScale;

	/** Option to apply some aggressive sleep logic, larger number is more agressive, 0 disables */
	UPROPERTY(EditAnywhere, Category = VehicleSetup)
	float SleepThreshold;
	
	/** Option to apply some aggressive sleep logic if slopes up Z is less than this value, i.e value = Cos(SlopeAngle) so 0.866 will sleep up to 30 degree slopes */
	UPROPERTY(EditAnywhere, Category = VehicleSetup, meta = (ClampMin = "0.01", UIMin = "0.01", ClampMax = "1.0", UIMax = "1.0"))
	float SleepSlopeLimit;

	/** Optional aerofoil setup - can be used for car spoilers or aircraft wings/elevator/rudder */
	UPROPERTY(EditAnywhere, Category = AerofoilSetup)
	TArray<FVehicleAerofoilConfig> Aerofoils;

	/** Optional thruster setup, use one or more as your main engine or as supplementary booster */
	UPROPERTY(EditAnywhere, Category = ThrusterSetup)
	TArray<FVehicleThrustConfig> Thrusters;

	/** Arcade style direct control of vehicle rotation via torque force */
	UPROPERTY(EditAnywhere, Category = ArcadeControl)
	FVehicleTorqueControlConfig TorqueControl;
	
	/** Arcade style direct control of vehicle rotation via torque force */
	UPROPERTY(EditAnywhere, Category = ArcadeControl)
	FVehicleTargetRotationControlConfig TargetRotationControl;

	/** Arcade style control of vehicle */
	UPROPERTY(EditAnywhere, Category = ArcadeControl)
	FVehicleStabilizeControlConfig StabilizeControl;

	// Used to recreate the physics if the blueprint changes.
	uint32 VehicleSetupTag;

protected:
	// True if the player is holding the handbrake
	UPROPERTY(Transient)
	uint8 bRawHandbrakeInput : 1;

	// True if the player is holding gear up
	UPROPERTY(Transient)
	uint8 bRawGearUpInput : 1;

	// True if the player is holding gear down
	UPROPERTY(Transient)
	uint8 bRawGearDownInput : 1;

	/** Was avoidance updated in this frame? */
	UPROPERTY(Transient)
	uint32 bWasAvoidanceUpdated : 1;


	Chaos::ETransmissionType TransmissionType;

public:

	/** UObject interface */
	virtual void Serialize(FArchive& Ar) override;
	/** End UObject interface*/

#if WITH_EDITOR
	/** Respond to a property change in editor */
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif //WITH_EDITOR

	/** Overridden to allow registration with components NOT owned by a Pawn. */
	virtual void SetUpdatedComponent(USceneComponent* NewUpdatedComponent) override;

	/** Allow the player controller of a different pawn to control this vehicle */
	virtual void SetOverrideController(AController* OverrideController);


	/** Return true if it's suitable to create a physics representation of the vehicle at this time */
	virtual bool ShouldCreatePhysicsState() const override;

	/** Returns true if the physics state exists */
	virtual bool HasValidPhysicsState() const override;

	/** Return true if we are ready to create a vehicle, false if the setup has missing references */
	virtual bool CanCreateVehicle() const;

	/** Used to create any physics engine information for this component */
	virtual void OnCreatePhysicsState() override;

	/** Used to shut down and physics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;

	/** Updates the vehicle tuning and other state such as user input. */
	virtual void PreTickGT(float DeltaTime);

	/** Stops movement immediately (zeroes velocity, usually zeros acceleration for components with acceleration). */
	virtual void StopMovementImmediately() override;


	/** Set the user input for the vehicle throttle [range 0 to 1] */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	void SetThrottleInput(float Throttle);

	/** Increase the vehicle throttle position [throttle range normalized 0 to 1] */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	void IncreaseThrottleInput(float ThrottleDelta);

	/** Decrease the vehicle throttle position  [throttle range normalized 0 to 1] */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	void DecreaseThrottleInput(float ThrottleDelta);

	/** Set the user input for the vehicle Brake [range 0 to 1] */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	void SetBrakeInput(float Brake);
	
	/** Set the user input for the vehicle steering [range -1 to 1] */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	void SetSteeringInput(float Steering);

	/** Set the user input for handbrake */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	void SetHandbrakeInput(bool bNewHandbrake);

	/** Set the vehicle sleeping (bEnableSleep=true) or wake it up (bEnableSleep=false) */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	void SetSleeping(bool bEnableSleep);

	/** Set the user input for gear up */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	void SetChangeUpInput(bool bNewGearUp);

	/** Set the user input for gear down */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	void SetChangeDownInput(bool bNewGearDown);

	/** Set the user input for gear (-1 reverse, 0 neutral, 1+ forward)*/
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	virtual void SetTargetGear(int32 GearNum, bool bImmediate);

	UFUNCTION(BlueprintCallable,BlueprintPure)
	virtual bool IsOnAnySurface(const TArray<UPhysicalMaterial*> surfaces);
	/** Set the flag that will be used to select auto-gears */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	void SetUseAutomaticGears(bool bUseAuto);

	/** Set the flag that determines whether a controller is required to set control inputs */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	void SetRequiresControllerForInputs(bool bRequiresController);

	/** Get current gear */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	virtual int32 GetCurrentGear() const;

	/** Get target gear */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	int32 GetTargetGear() const;

	/** Are gears being changed automatically? */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	bool GetUseAutoGears() const;

	/** How fast the vehicle is moving forward */
	UFUNCTION(BlueprintCallable, Category="Game|Components|ChaosVehicleMovement")
	float GetForwardSpeed() const;

	/** How fast the vehicle is moving forward */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	float GetForwardSpeedMPH() const;

	/** Get the user input for the vehicle throttle - can use this to feed control to a connected trailer */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	float GetThrottleInput() { return RawThrottleInput; }

	/** Get the user input for the vehicle brake - can use this to feed control to a connected trailer */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	float GetBrakeInput() { return RawBrakeInput; }

	/** Get the user input for the vehicle handbrake - can use this to feed control to a connected trailer */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	bool GetHandbrakeInput() const { return bRawHandbrakeInput; }

	/** Get the user input for the vehicle steering - can use this to feed control to a connected trailer */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	float GetSteeringInput() { return RawSteeringInput; }


	/** Reset some vehicle state - call this if you are say creating pool of vehicles that get reused and you don't want to carry over the previous state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	void ResetVehicle() { ResetVehicleState(); }

	/** Grab a snapshot of the vehicle instance dynamic state */
	void GetBaseSnapshot(FBaseSnapshotData& SnapshotOut) const;

	/** Set snapshot of vehicle instance dynamic state */
	void SetBaseSnapshot(const FBaseSnapshotData& SnapshotIn);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|ChaosVehicleMovement")
	void EnableSelfRighting(bool InState)
	{
		TargetRotationControl.Enabled = InState;
		TorqueControl.Enabled = InState;
		StabilizeControl.Enabled = InState;
	}

	/** location local coordinates of named bone in skeleton, apply additional offset or just use offset if no bone located */
	FVector LocateBoneOffset(const FName InBoneName, const FVector& InExtraOffset) const;

	virtual float GetSuspensionOffset(int WheelIndex) { return 0.f; }

	virtual void Update(float DeltaTime);

	virtual void ResetVehicleState();

	// Get output data from Physics Thread
	virtual void ParallelUpdate(float DeltaSeconds);

	float OutputInterpAlpha;

	struct FAsyncOutputWrapper
	{
		int32 Idx;
		int32 Timestamp;

		FAsyncOutputWrapper()
			: Idx(INDEX_NONE)
			, Timestamp(INDEX_NONE)
		{
		}
	};
	TArray<FAsyncOutputWrapper> OutputsWaitingOn;

protected:

	// replicated state of vehicle 
	UPROPERTY(Transient, Replicated)
	FVehicleReplicatedState ReplicatedState;

	// accumulator for RB replication errors 
	float AngErrorAccumulator;

	// What the player has the steering set to. Range -1...1
	UPROPERTY(Transient)
	float RawSteeringInput;

	// What the player has the accelerator set to. Range -1...1
	UPROPERTY(Transient)
	float RawThrottleInput;

	// What the player has the brake set to. Range -1...1
	UPROPERTY(Transient)
	float RawBrakeInput;

	// Steering output to physics system. Range -1...1
	UPROPERTY(Transient)
	float SteeringInput;

	// Accelerator output to physics system. Range 0...1
	UPROPERTY(Transient)
	float ThrottleInput;

	// Brake output to physics system. Range 0...1
	UPROPERTY(Transient)
	float BrakeInput;

	// Handbrake output to physics system. Range 0...1
	UPROPERTY(Transient)
	bool HandbrakeInput;

	// Bypass the need for a controller in order for the controls to be processed.
	UPROPERTY(EditAnywhere, Category=VehicleInput)
	bool bRequiresControllerForInputs;

	// How much to press the brake when the player has release throttle
	UPROPERTY(EditAnywhere, Category=VehicleInput)
	float IdleBrakeInput;

	// Auto-brake when absolute vehicle forward speed is less than this (cm/s)
	UPROPERTY(EditAnywhere, Category=VehicleInput)
	float StopThreshold;

	// Auto-brake when vehicle forward speed is opposite of player input by at least this much (cm/s)
	UPROPERTY(EditAnywhere, Category = VehicleInput)
	float WrongDirectionThreshold;

	// Rate at which input throttle can rise and fall
	UPROPERTY(EditAnywhere, Category=VehicleInput, AdvancedDisplay)
	FVehicleInputRateConfig ThrottleInputRate;

	// Rate at which input brake can rise and fall
	UPROPERTY(EditAnywhere, Category=VehicleInput, AdvancedDisplay)
	FVehicleInputRateConfig BrakeInputRate;

	// Rate at which input steering can rise and fall
	UPROPERTY(EditAnywhere, Category=VehicleInput, AdvancedDisplay)
	FVehicleInputRateConfig SteeringInputRate;

	// Rate at which input handbrake can rise and fall
	UPROPERTY(EditAnywhere, Category=VehicleInput, AdvancedDisplay)
	FVehicleInputRateConfig HandbrakeInputRate;

	// Rate at which input pitch can rise and fall
	UPROPERTY(EditAnywhere, Category = VehicleInput, AdvancedDisplay)
	FVehicleInputRateConfig PitchInputRate;

	// Rate at which input roll can rise and fall
	UPROPERTY(EditAnywhere, Category = VehicleInput, AdvancedDisplay)
	FVehicleInputRateConfig RollInputRate;

	// Rate at which input yaw can rise and fall
	UPROPERTY(EditAnywhere, Category = VehicleInput, AdvancedDisplay)
	FVehicleInputRateConfig YawInputRate;


	// input related

	/** Compute steering input */
	float CalcSteeringInput();

	/** Compute throttle & brake input */
	void CalcThrottleBrakeInput(float& ThrottleOut, float& BrakeOut);

	/** Compute handbrake input */
	bool CalcHandbrakeInput();

	/** Compute throttle inputs */
	float CalcThrottleUpInput();
	float CalcThrottleDownInput();

	/**
	* Clear all interpolated inputs to default values.
	* Raw input won't be cleared, the vehicle may resume input based movement next frame.
	*/
	virtual void ClearInput();
	
	/**
	* Clear all raw inputs to default values.
	* Interpolated input won't be cleared, the vehicle will begin interpolating to no input.
	*/
	virtual void ClearRawInput();

	void ClearAllInput()
	{
		ClearRawInput();
		ClearInput();
	}

	// Update

	/** Read current state for simulation */
	virtual void UpdateState(float DeltaTime);

	virtual void UpdateGroundedState(float DeltaTime) {};
	/** Option to aggressively sleep the vehicle */
	virtual void ProcessSleeping(const FControlInputs& ControlInputs);

	/** Pass current state to server */
	UFUNCTION(reliable, server, WithValidation)
	void ServerUpdateState(float InSteeringInput, float InThrottleInput, float InBrakeInput
			, bool InHandbrakeInput, int32 InCurrentGear);


	// Setup

	/** Get our controller */
	virtual AController* GetController() const override;

	/** Get the mesh this vehicle is tied to */
	class UMeshComponent* GetMesh() const;

	/** Get Mesh cast as USkeletalMeshComponent, may return null if cast fails */
	USkeletalMeshComponent* GetSkeletalMesh();

	/** Get Mesh cast as UStaticMeshComponent, may return null if cast fails */
	UStaticMeshComponent* GetStaticMesh();

	/** Create and setup the Chaos vehicle */
	virtual void CreateVehicle();


	/** Skeletal mesh needs some special handling in the vehicle case */
	virtual void FixupSkeletalMesh() {}

	/** Do some final setup after the Chaos vehicle gets created */
	virtual void PostSetupVehicle();

	/** Adjust the Chaos Physics mass */
	virtual void SetupVehicleMass();

	void UpdateMassProperties(FBodyInstance* BI);

	/** When vehicle is created we want to compute some helper data like drag area, etc.... Derived classes should use this to properly compute things like engine RPM */
	virtual void ComputeConstants();

	// Debug

	void ShowDebugInfo(class AHUD* HUD, class UCanvas* Canvas, const class FDebugDisplayInfo& DisplayInfo, float& YL, float& YPos);

	/** Draw debug text for the wheels and suspension */
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos);

	// draw 2D debug line to UI canvas
	void DrawLine2D(UCanvas* Canvas, const FVector2D& StartPos, const FVector2D& EndPos, FColor Color, float Thickness = 1.f);

	float GetForwardAcceleration()
	{
		return VehicleState.ForwardsAcceleration;
	}

	FBodyInstance* GetBodyInstance();
	const FBodyInstance* GetBodyInstance() const;

	/** Handle for delegate registered on mesh component */
	FDelegateHandle MeshOnPhysicsStateChangeHandle;

protected:

	virtual TObjectPtr<UChaosVehicleSimulation> GetVehicleSimulationPT() {return nullptr;}	/* simulation code running on the physics thread async callback */

	UPROPERTY(transient, Replicated)
	TObjectPtr<AController> OverrideController;

	const Chaos::FSimpleAerodynamicsConfig& GetAerodynamicsConfig()
	{
		FillAerodynamicsSetup();
		return PAerodynamicsSetup;
	}

	void FillAerodynamicsSetup()
	{
		PAerodynamicsSetup.DragCoefficient = this->DragCoefficient;
		PAerodynamicsSetup.DownforceCoefficient = this->DownforceCoefficient;
		PAerodynamicsSetup.AreaMetresSquared = Chaos::Cm2ToM2(this->DragArea);
	}

	void WakeAllEnabledRigidBodies();
	void PutAllEnabledRigidBodiesToSleep();

	Chaos::FSimpleAerodynamicsConfig PAerodynamicsSetup;
	int32 TargetGear;

	inline static EDebugPages DebugPage =EDebugPages::BasicPage;
	float PrevSteeringInput;
	float PrevReplicatedSteeringInput;
public:
	FVehicleState VehicleState;							/* Useful vehicle state captured at start of frame */
};
