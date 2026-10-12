// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "VehicleSurfaceStructs.h"
#include "VehicleSurfaceDataAsset.generated.h"

/**
* Data-driven road surface table.
* Maps physical materials (or physical surface types) to grip and rolling resistance.
* A new asset comes pre-filled with presets for common road types
* (highway asphalt, interstate concrete, city asphalt, farm chip seal, wet asphalt, gravel, dirt, desert sand, snow);
* assign your landscape / road physical materials to each entry.
*/
UCLASS(BlueprintType)
class KINETIFORGE_API UVehicleSurfaceDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UVehicleSurfaceDataAsset();

	/**
	* Used when the wheel hits something that is not mapped below.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Surface")
	FVehicleSurfaceProperties DefaultSurface;

	/**
	* What any surface turns into as the weather's SnowCover goes to 1.
	* Tires can tune it through SurfaceGripMultipliers with this SurfaceName (e.g. winter tires: Snow = 1.5).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Surface")
	FVehicleSurfaceProperties SnowSurface;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Surface")
	TArray<FVehicleSurfaceEntry> Surfaces;

	/**
	* If true, a mapped surface ignores the physical material's own Friction value and the tire's friction combine mode:
	* grip = tire Global Friction Scale * GripScale.
	* If false, GripScale multiplies the result of the tire's friction combine mode.
	* Unmapped surfaces always use the combine mode (times DefaultSurface.GripScale).
	*/
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Surface")
	bool bIgnorePhysicalMaterialFriction = true;

	/**
	* Builds the physics-thread lookup table. Later entries do not override earlier ones.
	* TireGripMultipliers (keyed by SurfaceName) lets one tire grip differently on a surface, e.g. off-road tires on gravel.
	*/
	TSharedRef<const FVehicleSurfaceLookup, ESPMode::ThreadSafe> BuildLookup(const TMap<FName, float>* TireGripMultipliers = nullptr) const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Surface")
	FVehicleSurfaceProperties FindSurface(const UPhysicalMaterial* PhysMaterial) const;

	static FVehicleSurfaceProperties MakeDryAsphalt();
	static FVehicleSurfaceProperties MakeWetAsphalt();
	static FVehicleSurfaceProperties MakeGravel();
	static FVehicleSurfaceProperties MakeDirt();
	static FVehicleSurfaceProperties MakeSnow();
	static FVehicleSurfaceProperties MakeConcrete();
	static FVehicleSurfaceProperties MakeCityAsphalt();
	static FVehicleSurfaceProperties MakeChipSeal();
	static FVehicleSurfaceProperties MakeSand();
};
