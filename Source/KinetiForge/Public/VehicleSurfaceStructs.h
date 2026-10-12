// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "VehicleSurfaceStructs.generated.h"

/**
* How a road surface changes tire behavior.
* All scales are relative to the tire's own config, so 1.0 means "same as the tire was tuned for".
*/
USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleSurfaceProperties
{
	GENERATED_BODY()

	/**
	* Name reported in the wheel state (useful for audio, particles and debugging).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName SurfaceName = TEXT("Default");

	/**
	* Overall grip (peak friction) relative to the tire's Global Friction Scale.
	* Dry asphalt = 1.0.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float GripScale = 1.f;

	/**
	* Extra scale on longitudinal grip (traction and braking), on top of GripScale.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float LongitudinalGripScale = 1.f;

	/**
	* Extra scale on lateral grip (cornering), on top of GripScale.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float LateralGripScale = 1.f;

	/**
	* Dimensionless rolling resistance coefficient (Crr).
	* Rolling resistance torque = Crr * WheelLoad * Radius, applied on top of the wheel's constant RollingRisistance.
	* Typical: asphalt 0.010-0.015, gravel 0.02-0.03, dirt 0.03-0.06, sand 0.1-0.3.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0"))
	float RollingResistanceCoefficient = 0.f;

	/**
	* Scales the tire relaxation length. Loose surfaces (gravel, dirt) build up force more slowly, so use > 1.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.01"))
	float RelaxationLengthScale = 1.f;
};

/**
* One surface and the physical materials / surface types that map to it.
*/
USTRUCT(BlueprintType)
struct KINETIFORGE_API FVehicleSurfaceEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVehicleSurfaceProperties Properties;

	/**
	* Physical materials that use this surface. Checked before SurfaceTypes.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<TObjectPtr<UPhysicalMaterial>> PhysicalMaterials;

	/**
	* Physical surface types (Project Settings > Physics > Physical Surface) that use this surface.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<TEnumAsByte<EPhysicalSurface>> SurfaceTypes;
};

/**
* Flattened, read-only surface table used by the wheel solver on the physics thread.
* Built on the game thread from a UVehicleSurfaceDataAsset and never modified afterwards.
*/
struct KINETIFORGE_API FVehicleSurfaceLookup
{
	static constexpr int32 NumSurfaceTypes = (int32)SurfaceType_Max;

	bool bIgnorePhysicalMaterialFriction = true;

	FVehicleSurfaceProperties DefaultSurface;

	TArray<FVehicleSurfaceProperties> Surfaces;

	TMap<const UPhysicalMaterial*, int32> MaterialToSurface;

	int32 SurfaceTypeToSurface[NumSurfaceTypes];

	FVehicleSurfaceLookup()
	{
		for (int32 i = 0; i < NumSurfaceTypes; i++)
		{
			SurfaceTypeToSurface[i] = INDEX_NONE;
		}
	}

	/**
	* Returns the index into Surfaces, or INDEX_NONE if the material is not mapped (use DefaultSurface).
	*/
	int32 FindSurfaceIndex(const UPhysicalMaterial* PhysMaterial) const
	{
		if (!PhysMaterial)
		{
			return INDEX_NONE;
		}

		if (const int32* Found = MaterialToSurface.Find(PhysMaterial))
		{
			return *Found;
		}

		const int32 SurfaceType = (int32)PhysMaterial->SurfaceType.GetValue();
		if (SurfaceType >= 0 && SurfaceType < NumSurfaceTypes)
		{
			return SurfaceTypeToSurface[SurfaceType];
		}

		return INDEX_NONE;
	}

	const FVehicleSurfaceProperties& GetSurface(const int32 SurfaceIndex) const
	{
		return Surfaces.IsValidIndex(SurfaceIndex) ? Surfaces[SurfaceIndex] : DefaultSurface;
	}
};
