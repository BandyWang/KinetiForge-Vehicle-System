// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "VehicleUtilities.h"
#include "VehicleWheelStructs.generated.h"

UENUM(BlueprintType)
enum class ETireFrictionCombineMode : uint8
{
	Constant,
	Average,
	Multiply,
	Min,
	Max
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleWheelConfig
{
	GENERATED_BODY()

	/*Unit: cm*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Radius = 33.f;

	/*Unit: cm*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Width = 20.f;

	/*Unit: kg*m^2*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Inertia = 1.f;

	/**
	* This is just a constant additional handbrake torque
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float RollingRisistance = 5.f;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleTireThermalConfig
{
	GENERATED_BODY()

	/**
	* Simulates tread and core temperature, and pressure, and scales grip with them.
	* When enabled, the road-temperature grip settings in the Surface category are ignored:
	* road temperature then reaches the grip through the tire temperature instead.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bEnableThermalModel = false;

	/**
	* Starting temperature (C) of the tire. Below -273 = start at the air temperature.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float InitialTemperature = -300.f;

	/**
	* Temperature (C) with full grip. Road tires ~50-70, race slicks ~85-100.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip")
	float OptimalTemperature = 60.f;

	/**
	* Degrees below optimal where grip reaches MinGripScale. Grip falls off quadratically.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip", meta = (ClampMin = "1.0"))
	float ColdWindow = 80.f;

	/**
	* Degrees above optimal where grip reaches MinGripScale.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip", meta = (ClampMin = "1.0"))
	float HotWindow = 60.f;

	/**
	* Lowest grip multiplier temperature and pressure can cause together.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinGripScale = 0.75f;

	/**
	* How much the tread surface temperature counts for grip; the rest is the core temperature.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float SurfaceTemperatureGripWeight = 0.7f;

	/**
	* Gauge pressure (bar) set when the tire is at ColdPressureTemperature.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pressure", meta = (ClampMin = "0.0"))
	float ColdPressure = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pressure")
	float ColdPressureTemperature = 20.f;

	/**
	* Hot gauge pressure (bar) with full grip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pressure", meta = (ClampMin = "0.0"))
	float OptimalPressure = 2.3f;

	/**
	* Grip lost per bar away from OptimalPressure. 0 = pressure has no effect on grip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pressure", meta = (ClampMin = "0.0"))
	float PressureGripSensitivity = 0.05f;

	/**
	* Unit: J/K. Heat capacity of the tread surface layer.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "1.0"))
	float SurfaceHeatCapacity = 3000.f;

	/**
	* Unit: J/K. Heat capacity of the carcass and air.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "1.0"))
	float CoreHeatCapacity = 15000.f;

	/**
	* Share of the sliding friction power (force * slip speed) that goes into the tread. The rest heats the road.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float SlipHeatFraction = 0.6f;

	/**
	* Carcass flexing heat, as a fraction of wheel load * rolling speed (like a rolling resistance coefficient).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0"))
	float HysteresisHeatCoefficient = 0.008f;

	/**
	* Unit: W/K. Heat flow between tread and core.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0"))
	float SurfaceToCoreConductance = 40.f;

	/**
	* Unit: W/K. Tread cooling to the air when standing still.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0"))
	float AirCoolingBase = 5.f;

	/**
	* Unit: W/K per m/s. Extra tread cooling to the air with speed.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0"))
	float AirCoolingPerSpeed = 0.6f;

	/**
	* Core (sidewall) cooling to the air, as a fraction of the tread air cooling.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0"))
	float CoreAirCoolingScale = 0.5f;

	/**
	* Unit: W/K. Heat flow between tread and road while touching it.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0"))
	float RoadConductance = 20.f;

	/**
	* Extra tread cooling on a fully wet road: road conductance is multiplied by (1 + this * Wetness).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Heat", meta = (ClampMin = "0.0"))
	float WetCoolingFactor = 4.f;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleBrakeThermalConfig
{
	GENERATED_BODY()

	/**
	* Simulates disc temperature and scales brake torque with it (brake fade).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bEnableThermalModel = false;

	/**
	* Starting disc temperature (C). Below -273 = start at the air temperature.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float InitialTemperature = -300.f;

	/**
	* Unit: J/K. Disc mass * specific heat. Cast iron is ~500 J/(kg*K), so a 9 kg disc is ~4500.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "1.0"))
	float HeatCapacity = 4500.f;

	/**
	* Unit: W/K. Cooling when standing still.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float CoolingBase = 3.f;

	/**
	* Unit: W/K per m/s. Extra cooling with speed (vented discs and ducts cool faster).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float CoolingPerSpeed = 0.6f;

	/**
	* Friction multiplier of a cold brake (at air temperature).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float ColdFrictionScale = 0.9f;

	/**
	* Temperature (C) where the pads reach full friction.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float OptimalMinTemperature = 150.f;

	/**
	* Temperature (C) where fade begins. Street pads ~400, race pads ~650.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float FadeStartTemperature = 450.f;

	/**
	* Temperature (C) where fade bottoms out at FadedFrictionScale.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float FadeEndTemperature = 700.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float FadedFrictionScale = 0.5f;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleTireWearConfig
{
	GENERATED_BODY()

	/**
	* Tread wears with sliding energy and overheating; worn or blown tires lose grip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bEnableWear = false;

	/**
	* Tread used per megajoule of sliding friction work at the contact patch.
	* 0.05 = a new tire is gone after 20 MJ (roughly a few hard track sessions).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float WearPerMegajoule = 0.05f;

	/**
	* Dry grip multiplier of a fully worn tire.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WornGripScale = 0.85f;

	/**
	* Extra grip lost by a fully worn tire on a fully wet road (no tread to clear water).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WetWornGripLoss = 0.5f;

	/**
	* Tread surface temperature (C) above which the rubber blisters and wear speeds up. Needs the thermal model.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float OverheatTemperature = 130.f;

	/**
	* Wear is multiplied by (1 + OverheatWearMultiplier) for every OverheatWindow degrees above OverheatTemperature.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float OverheatWearMultiplier = 4.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "1.0"))
	float OverheatWindow = 50.f;

	/**
	* Tread surface can't get hotter than this (C): past it the rubber burns off (smoke) instead.
	* The excess heat is turned into wear. Needs the thermal model.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float MaxTreadTemperature = 250.f;

	/**
	* Megajoules of excess heat above MaxTreadTemperature that burn off the whole tread.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.001"))
	float BurnOffMegajoules = 2.f;

	/**
	* The tire blows out above this gauge pressure (bar). 0 = never. Needs the thermal model.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float BlowoutPressure = 4.f;

	/**
	* Grip multiplier of a blown tire (running on the carcass or rim).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BlownGripScale = 0.3f;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleTireConfig
{
	GENERATED_BODY()

	/**
	* Overall grip multiplier for this tire.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Global Friction Scale"))
	float FrictionMultiplier = 1.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ETireFrictionCombineMode TireFrictionCombineMode = ETireFrictionCombineMode::Average;

	/**
	* Simulates tire lag. Higher values = softer/laggy response.
	* Increase this if the vehicle jitters at low frame rates.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Relaxation Length"))
	FVector2f RelaxationLength = FVector2f(0.1f, 0.2f);

	/**
	* if value is 1, the wheel load will be proportional to suspension force. 
	* If value is 0, the wheel load will be constant (gravity of sprungmass).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float WheelLoadInfluenceFactor = 0.8f;

	/**
	* Balances grip between turning and accelerating.
	* 0.5 = Balanced.
	* >0.5 = Prioritizes Turning (Easier to corner while braking).
	* <0.5 = Prioritizes Acceleration.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Grip Distribution Bias"))
	float CombinedSlipBias = 0.5f;

	/**
	* How much longitudinal slip (spinning) reduces lateral grip.
	* 1.0 = Realistic (Spinning wheels can't turn).
	* 0.0 = Arcade (Full turning grip even when burning out).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Accel Affects Turning"))
	float LongitudinalToLateralInterference = 1.f;

	/**
	* How much lateral slip (drifting) reduces forward acceleration.
	* Lower values (e.g., 0.8) make it easier to maintain speed while drifting.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Turning Affects Accel"))
	float LateralToLongitudinalInterference = 1.f;

	/**
	* Scales the Fx curve output. Determines max longitudinal force.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Longitudinal Grip Limit"))
	float MaxFx = 1.f;

	/**
	* Scales the Fy curve output. Determines max cornering force.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", DisplayName = "Lateral Grip Limit"))
	float MaxFy = 1.f;

	/**
	* Input: Slip Ratio. Output: Friction Coefficient.
	* Note: Slope at origin will be used to determine Optimal Slip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Curve: Fx (Longitudinal)"))
	UCurveFloat* Fx = nullptr;

	/**
	* Input: Slip Angle (Deg). Output: Friction Coefficient.
	* Note: Slope at origin will be used to determine Optimal Slip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Curve: Fy (Lateral)"))
	UCurveFloat* Fy = nullptr;

	/**
	* Input: abs camber angle in degrees.
	* Output: lateral drift per rolling distance, dy / dx.
	*
	* This is not a force scale.
	* This is not a constant lateral velocity.
	*
	* The solver converts it to lateral slip velocity:
	*
	*     CamberSlipVelocityY = RollSpeed * CamberLateralDrift
	*
	* Example:
	*     Camber = 5 deg
	*     Curve output = 0.01
	*     means the contact patch tends to drift 1 cm laterally per 1 m rolling distance.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Curve: Camber To Lateral Drift"))
	UCurveFloat* CamberToLateralDrift = nullptr;

	/**
	* Per-surface grip multiplier for this tire, keyed by the surface's SurfaceName.
	* E.g. an off-road tire could use Gravel = 1.2, a slick Gravel = 0.7.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface")
	TMap<FName, float> SurfaceGripMultipliers;

	/**
	* Road temperature (C) at which this tire has full grip.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface")
	float OptimalRoadTemperature = 25.f;

	/**
	* Grip lost per degree C away from OptimalRoadTemperature. 0 = temperature has no effect.
	* E.g. 0.004 loses 10% grip 25 C away from optimal.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ClampMin = "0.0"))
	float RoadTemperatureGripFalloff = 0.f;

	/**
	* Lowest grip multiplier temperature alone can cause.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinTemperatureGripScale = 0.7f;

	/**
	* Unit: cm. Distance behind the contact patch center where lateral force acts, at zero slip and static load.
	* Shrinks to 0 as the tire reaches its grip peak, which is the "going light" feel in the steering.
	* Typical: 2-5 cm. 0 = no self-aligning torque.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aligning Torque", meta = (ClampMin = "0.0"))
	float PneumaticTrail = 3.f;

	/**
	* Slip angle (deg) of the grip peak, used for the trail falloff when no Fy curve is set.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aligning Torque", meta = (ClampMin = "0.1"))
	float FallbackPeakSlipAngle = 8.f;

	/**
	* Also apply the self-aligning torque to the chassis, as a rigid steering rack does.
	* Off: it is only reported (for force feedback).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Aligning Torque")
	bool bApplyAligningTorqueToChassis = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thermal")
	FVehicleTireThermalConfig Thermal;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wear")
	FVehicleTireWearConfig Wear;
};

USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleABSConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bAntiBrakeSystemEnabled = true;

	/**
	* The target slip ratio the anti-brake-system try to maintain.
	* The value is dependent on your tire config and the road condition.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float OptimalSlip = 0.1f;

	/**
	* Minimum speed required to activate anti-brake-system.
	* Unit: meter per second, m/s
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float ActivationSpeed = 5.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float Sensitivity = 5.f;
};

template <int32 NumSamples>
struct KINETIFORGE_API FVehicleTireLUT : public FVehicleLUT<NumSamples>
{
public:
	// True, if can be used for combined slip
	bool bHasValidStiffness = false;

	int32 OptimalSlipIndex = 0;
	float LinearStiffness = 0.f;
	float PeakFriction = 0.f;

	FVehicleTireLUT(const float InitValue = 0.f) : FVehicleLUT<NumSamples>(InitValue)
	{
	}

	FVehicleTireLUT(const FRichCurve& RichCurve, const FVector2f SelectedTimeInterval = FVector2f(0.f, 1.f))
	{
		BuildFromCurve(RichCurve, SelectedTimeInterval);
	}

	void BuildFromCurve(const FRichCurve& RichCurve, const FVector2f SelectedTimeInterval = FVector2f(0.f, 1.f))
	{
		// Call the superclass method to copy the data and populate the `Samples` array
		this->CopyFromRichCurve(RichCurve, SelectedTimeInterval);

		bHasValidStiffness = false;
		LinearStiffness = 0.f;
		PeakFriction = 0.f;

		if (RichCurve.Keys.Num() == 0) return;

		const float TimeMin = SelectedTimeInterval.X;
		const float TimeMax = SelectedTimeInterval.Y;
		const float IntervalLength = TimeMax - TimeMin;

		if (IntervalLength <= SMALL_NUMBER) return;

		// Constrain the negative force and find the global maximum grip force
		for (int32 i = 0; i < NumSamples; i++)
		{
			if (this->Samples[i] < 0.f)
			{
				this->Samples[i] = 0.f; // Forcefully Compensate for Negative Grip
			}

			if (this->Samples[i] > PeakFriction)
			{
				PeakFriction = this->Samples[i];
			}
		}

		const float StartValue = this->Samples[0];
		const float EndValue = this->Samples[NumSamples - 1];

		// Finding the Optimal Slip Using the Maximum Deviation Method
		const float SecantSlope = (EndValue - StartValue) / (float)(NumSamples - 1);

		float MaxDeviation = -1.f;
		int32 OptimalLocalIndex = 0;

		for (int32 i = 0; i < NumSamples; i++)
		{
			float CurrentY = this->Samples[i];
			float SecantY = StartValue + SecantSlope * i;
			float Deviation = CurrentY - SecantY;

			if (Deviation > MaxDeviation)
			{
				MaxDeviation = Deviation;
				OptimalLocalIndex = i;
			}
		}

		// cache optimal slip
		OptimalSlipIndex = OptimalLocalIndex;

		// try to find stiffness of linear region
		float MaxLocalStiffness = 0.f;
		for (int32 i = 0; i < OptimalSlipIndex; i++)
		{
			float CurrentY = this->Samples[i];
			float NextY = this->Samples[i + 1];
			float LocalStiffness = (NextY - CurrentY) * (float)(NumSamples - 1);

			if (LocalStiffness > MaxLocalStiffness)
			{
				MaxLocalStiffness = LocalStiffness;
			}
		}

		LinearStiffness = MaxLocalStiffness;

		// Check if stiffness is valid
		if (LinearStiffness > SMALL_NUMBER)
		{
			bHasValidStiffness = true;
		}
	}
};

USTRUCT()
struct KINETIFORGE_API FVehicleWheelCachedLUTs
{
	GENERATED_BODY()

	FVehicleTireLUT<64> Fx = FVehicleTireLUT<64>(1.f);
	FVehicleTireLUT<64> Fy = FVehicleTireLUT<64>(1.f);
	FVehicleLUT<64> CamberToLateralDrift = FVehicleLUT<64>(0.f);
};

USTRUCT(BlueprintType, meta = (ToolTip = "wheel state in simulation"))
struct KINETIFORGE_API FVehicleWheelSimState
{
	GENERATED_BODY()
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float EffectiveInertia = 1.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float AngularVelocity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float AngularAcceleration = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float LongSlipVelocity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float LatSlipVelocity = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float SlipRatio = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float SlipAngle = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	float PredictedSlipRatio = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float SignedCamberDegree = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	float DynFrictionMultiplier = 1.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Surface")
	FName SurfaceName = NAME_None;
	/** Index into the surface data asset's Surfaces array, -1 for the default surface */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Surface")
	int32 SurfaceIndex = INDEX_NONE;
	/** Rolling resistance torque from the surface, included in BrakeTorque */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Surface")
	float SurfaceRollingResistanceTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	FVector2f LocalLinearVelocity = FVector2f(0.f, 0.f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Force")
	float WheelLoad = 0.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Force")
	float P4MotorTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float DriveTorque = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorque = 0.f;		//brake torque from brake + rolling resistance
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorqueFromBrake = 0.f;	//brake torque input + brake torque from esp; not including the rolling resistance
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorqueFromHandbrake = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float BrakeTorqueFromESP = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	float TorqueFromGroundInteraction = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Slip")
	FVector2f TransientSlip = FVector2f(0.f);
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	FVector2f TireForce2D = FVector2f(0.f);
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Force")
	FVector3f TireForce = FVector3f(0.f, 0.f, 0.f);
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Movement")
	bool bIsLocked = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Electronics")
	bool bABSTriggered = false;

	/** Unit: cm. Current pneumatic trail */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Steering")
	float PneumaticTrail = 0.f;
	/** Unit: N*m. Self-aligning torque about the contact normal */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Steering")
	float AligningTorque = 0.f;
	/** Unit: N*m (world). Self-aligning torque as a vector */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Steering")
	FVector3f AligningTorqueVector = FVector3f(0.f);
	/**
	* Unit: N*m. Torque about the steering (kingpin) axis from all tire forces:
	* pneumatic + mechanical trail, scrub radius and weight jacking.
	* Same sign convention as the steering angle; it opposes the steering angle when self-centering.
	*/
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Steering")
	float SteeringAxisTorque = 0.f;

	/** Unit: C */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thermal")
	float TireSurfaceTemperature = 20.f;
	/** Unit: C */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thermal")
	float TireCoreTemperature = 20.f;
	/** Unit: bar (gauge) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thermal")
	float TirePressure = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thermal")
	float TireThermalGripScale = 1.f;
	/** Unit: C */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thermal")
	float BrakeTemperature = 20.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thermal")
	float BrakeFrictionScale = 1.f;

	/** 1 = new tread, 0 = worn down to the cords */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wear")
	float TreadRemaining = 1.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wear")
	bool bTireBlown = false;
	/** Grip multiplier from wear and blowout */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wear")
	float TireWearGripScale = 1.f;

	bool bTireThermalInitialized = false;
	bool bBrakeThermalInitialized = false;
};

USTRUCT(BlueprintType, meta = (ToolTip = ""))
struct KINETIFORGE_API FVehicleWheelSimContext
{
	GENERATED_BODY()

	float MacroDeltaTime = 1 / 120.f;
	float MacroDeltaTimeInv = 120.f;
	float SubstepDeltaTime = 1 / 120.f;
	float SubstepDeltaTimeInv = 120.f;

	float LongForceScale = 1.f;
	float LatForceScale = 1.f;
	FVector3f LongForceDir = FVector3f(0.f);
	FVector3f LatForceDir = FVector3f(0.f);

	float CamberLateralDrift = 0.f;

	float SurfaceLongGripScale = 1.f;
	float SurfaceLatGripScale = 1.f;
	float SurfaceRelaxationLengthScale = 1.f;

	float AvailableGrip = 0.f;
	FVector2f GravityComp2D = FVector2f(0.f);
	FVector2f AccumulateTireImpulse2D = FVector2f(0.f);
	// energy (J) over the macro step, for the thermal models
	float AccumulatedSlipEnergy = 0.f;
	float AccumulatedBrakeEnergy = 0.f;

	float R = 0.33f;
	float RInv = 1 / 0.33f;
};