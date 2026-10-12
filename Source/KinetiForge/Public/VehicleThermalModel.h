// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"

/**
* Plain math for the tire / brake thermal models and the pneumatic trail.
* No UObject dependencies, so it can be tested outside the engine.
*/
namespace VehicleThermalModel
{
	constexpr float KelvinOffset = 273.15f;
	constexpr float AtmosphereBar = 1.01325f;

	/**
	* One semi-implicit step of a lumped thermal node exchanging heat with up to three neighbours.
	* Unconditionally stable for any dt and conductance.
	* C: heat capacity (J/K), Q: heat input (W), G: conductances (W/K), TN: neighbour temperatures.
	*/
	FORCEINLINE float StepNode(
		const float T, const float C, const float Q, const float Dt,
		const float G0, const float T0,
		const float G1 = 0.f, const float T1 = 0.f,
		const float G2 = 0.f, const float T2 = 0.f)
	{
		const float SafeC = FMath::Max(C, 1.f);
		const float K = Dt / SafeC;
		const float SumG = FMath::Max(G0, 0.f) + FMath::Max(G1, 0.f) + FMath::Max(G2, 0.f);
		const float Source = Q + FMath::Max(G0, 0.f) * T0 + FMath::Max(G1, 0.f) * T1 + FMath::Max(G2, 0.f) * T2;
		return (T + K * Source) / (1.f + K * SumG);
	}

	/**
	* Grip scale for a temperature window.
	* 1 at Optimal, falls quadratically to MinScale at Optimal - ColdWindow and Optimal + HotWindow, flat beyond.
	*/
	FORCEINLINE float TemperatureWindowScale(
		const float T, const float Optimal, const float ColdWindow, const float HotWindow, const float MinScale)
	{
		const float Delta = T - Optimal;
		const float Window = FMath::Max(Delta < 0.f ? ColdWindow : HotWindow, 1.f);
		const float X = FMath::Min(FMath::Abs(Delta) / Window, 1.f);
		return 1.f - (1.f - FMath::Clamp(MinScale, 0.f, 1.f)) * X * X;
	}

	/** Gauge pressure (bar) of a fixed volume of gas filled to ColdGaugePressure at ColdTemperature, now at Temperature (C). */
	FORCEINLINE float GaugePressure(const float ColdGaugePressure, const float ColdTemperature, const float Temperature)
	{
		const float ColdAbs = ColdGaugePressure + AtmosphereBar;
		const float Ratio = FMath::Max(Temperature + KelvinOffset, 1.f) / FMath::Max(ColdTemperature + KelvinOffset, 1.f);
		return ColdAbs * Ratio - AtmosphereBar;
	}

	/** Grip scale vs gauge pressure: 1 at Optimal, falls by Sensitivity per bar away from it. */
	FORCEINLINE float PressureScale(const float Pressure, const float Optimal, const float Sensitivity, const float MinScale)
	{
		return FMath::Max(1.f - FMath::Max(Sensitivity, 0.f) * FMath::Abs(Pressure - Optimal), FMath::Clamp(MinScale, 0.f, 1.f));
	}

	/**
	* Brake friction scale vs disc temperature.
	* ColdScale at or below AmbientRef, ramps to 1 at OptimalMin, flat to FadeStart, ramps down to FadedScale at FadeEnd.
	*/
	FORCEINLINE float BrakeFrictionScale(
		const float T, const float AmbientRef, const float ColdScale, const float OptimalMin,
		const float FadeStart, const float FadeEnd, const float FadedScale)
	{
		if (T < OptimalMin)
		{
			const float Alpha = FMath::Clamp((T - AmbientRef) / FMath::Max(OptimalMin - AmbientRef, 1.f), 0.f, 1.f);
			return FMath::Lerp(ColdScale, 1.f, Alpha);
		}
		if (T <= FadeStart)
		{
			return 1.f;
		}
		const float Alpha = FMath::Clamp((T - FadeStart) / FMath::Max(FadeEnd - FadeStart, 1.f), 0.f, 1.f);
		return FMath::Lerp(1.f, FadedScale, Alpha);
	}

	/**
	* Wear multiplier from tread temperature: 1 up to OverheatTemperature,
	* then grows linearly by OverheatWearMultiplier for every OverheatWindow degrees above it (blistering, graining).
	*/
	FORCEINLINE float OverheatWearScale(
		const float T, const float OverheatTemperature, const float OverheatWindow, const float OverheatWearMultiplier)
	{
		const float Excess = FMath::Max(T - OverheatTemperature, 0.f) / FMath::Max(OverheatWindow, 1.f);
		return 1.f + FMath::Max(OverheatWearMultiplier, 0.f) * Excess;
	}

	/**
	* Grip scale from remaining tread (1 = new, 0 = down to the cords).
	* Dry grip falls toward WornGripScale; on a wet road a worn tire also can't clear water.
	*/
	FORCEINLINE float TreadGripScale(
		const float TreadRemaining, const float WornGripScale, const float Wetness, const float WetWornGripLoss)
	{
		const float Worn = 1.f - FMath::Clamp(TreadRemaining, 0.f, 1.f);
		const float Dry = FMath::Lerp(1.f, FMath::Clamp(WornGripScale, 0.f, 1.f), Worn);
		const float Wet = 1.f - FMath::Clamp(Wetness, 0.f, 1.f) * FMath::Clamp(WetWornGripLoss, 0.f, 1.f) * Worn;
		return Dry * Wet;
	}

	/**
	* Brush-model pneumatic trail ratio t / t0.
	* Lambda: how far the contact patch is into sliding, 0 = no slip, 1 = fully sliding (at the friction peak).
	*/
	FORCEINLINE float PneumaticTrailRatio(const float Lambda)
	{
		const float L = FMath::Clamp(Lambda, 0.f, 1.f);
		const float OneMinus = 1.f - L;
		return OneMinus * OneMinus * OneMinus / (1.f - L + L * L / 3.f);
	}
}
