// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "VehicleSurfaceStructs.h"
#include "VehicleSurfaceDataAsset.generated.h"

/**
* Data-driven road surface table.
* Maps physical materials (or physical surface types) to grip and rolling resistance.
* A new asset comes pre-filled with Dry Asphalt, Wet Asphalt, Gravel and Dirt;
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
	*/
	TSharedRef<const FVehicleSurfaceLookup, ESPMode::ThreadSafe> BuildLookup() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Surface")
	FVehicleSurfaceProperties FindSurface(const UPhysicalMaterial* PhysMaterial) const;

	static FVehicleSurfaceProperties MakeDryAsphalt();
	static FVehicleSurfaceProperties MakeWetAsphalt();
	static FVehicleSurfaceProperties MakeGravel();
	static FVehicleSurfaceProperties MakeDirt();
};
