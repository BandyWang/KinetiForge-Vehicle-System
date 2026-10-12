// Copyright (c) 2026 Zhengyi Miao (github.com/myoozy)


#include "VehicleSurfaceDataAsset.h"

UVehicleSurfaceDataAsset::UVehicleSurfaceDataAsset()
{
	DefaultSurface = MakeDryAsphalt();
	DefaultSurface.SurfaceName = TEXT("Default");

	FVehicleSurfaceEntry DryAsphalt;
	DryAsphalt.Properties = MakeDryAsphalt();
	Surfaces.Add(DryAsphalt);

	FVehicleSurfaceEntry WetAsphalt;
	WetAsphalt.Properties = MakeWetAsphalt();
	Surfaces.Add(WetAsphalt);

	FVehicleSurfaceEntry Gravel;
	Gravel.Properties = MakeGravel();
	Surfaces.Add(Gravel);

	FVehicleSurfaceEntry Dirt;
	Dirt.Properties = MakeDirt();
	Surfaces.Add(Dirt);
}

TSharedRef<const FVehicleSurfaceLookup, ESPMode::ThreadSafe> UVehicleSurfaceDataAsset::BuildLookup() const
{
	TSharedRef<FVehicleSurfaceLookup, ESPMode::ThreadSafe> Lookup = MakeShared<FVehicleSurfaceLookup, ESPMode::ThreadSafe>();

	Lookup->bIgnorePhysicalMaterialFriction = bIgnorePhysicalMaterialFriction;
	Lookup->DefaultSurface = DefaultSurface;

	for (const FVehicleSurfaceEntry& Entry : Surfaces)
	{
		const int32 SurfaceIndex = Lookup->Surfaces.Add(Entry.Properties);

		for (const TObjectPtr<UPhysicalMaterial>& PhysMaterial : Entry.PhysicalMaterials)
		{
			if (PhysMaterial && !Lookup->MaterialToSurface.Contains(PhysMaterial.Get()))
			{
				Lookup->MaterialToSurface.Add(PhysMaterial.Get(), SurfaceIndex);
			}
		}

		for (const TEnumAsByte<EPhysicalSurface>& SurfaceType : Entry.SurfaceTypes)
		{
			const int32 TypeIndex = (int32)SurfaceType.GetValue();
			// SurfaceType_Default is what every unconfigured material reports, so it cannot identify a surface
			if (TypeIndex > (int32)SurfaceType_Default
				&& TypeIndex < FVehicleSurfaceLookup::NumSurfaceTypes
				&& Lookup->SurfaceTypeToSurface[TypeIndex] == INDEX_NONE)
			{
				Lookup->SurfaceTypeToSurface[TypeIndex] = SurfaceIndex;
			}
		}
	}

	return Lookup;
}

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::FindSurface(const UPhysicalMaterial* PhysMaterial) const
{
	TSharedRef<const FVehicleSurfaceLookup, ESPMode::ThreadSafe> Lookup = BuildLookup();
	return Lookup->GetSurface(Lookup->FindSurfaceIndex(PhysMaterial));
}

// Grip values are peak friction relative to dry asphalt (dry asphalt mu ~0.9-1.0 for a road tire).
// Rolling resistance coefficients are typical passenger car tire values.

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::MakeDryAsphalt()
{
	FVehicleSurfaceProperties Props;
	Props.SurfaceName = TEXT("DryAsphalt");
	Props.GripScale = 1.f;
	Props.LongitudinalGripScale = 1.f;
	Props.LateralGripScale = 1.f;
	Props.RollingResistanceCoefficient = 0.012f;
	Props.RelaxationLengthScale = 1.f;
	return Props;
}

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::MakeWetAsphalt()
{
	FVehicleSurfaceProperties Props;
	Props.SurfaceName = TEXT("WetAsphalt");
	// wet mu ~0.5-0.8; no standing water (aquaplaning is not modeled)
	Props.GripScale = 0.7f;
	Props.LongitudinalGripScale = 1.f;
	Props.LateralGripScale = 1.f;
	Props.RollingResistanceCoefficient = 0.015f;
	Props.RelaxationLengthScale = 1.f;
	return Props;
}

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::MakeGravel()
{
	FVehicleSurfaceProperties Props;
	Props.SurfaceName = TEXT("Gravel");
	// loose stones roll under the tire, so cornering suffers more than traction
	Props.GripScale = 0.6f;
	Props.LongitudinalGripScale = 1.f;
	Props.LateralGripScale = 0.9f;
	Props.RollingResistanceCoefficient = 0.025f;
	Props.RelaxationLengthScale = 1.6f;
	return Props;
}

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::MakeDirt()
{
	FVehicleSurfaceProperties Props;
	Props.SurfaceName = TEXT("Dirt");
	// dry packed dirt; wet or loose dirt should get its own entry with lower grip and higher Crr
	Props.GripScale = 0.65f;
	Props.LongitudinalGripScale = 1.f;
	Props.LateralGripScale = 0.95f;
	Props.RollingResistanceCoefficient = 0.04f;
	Props.RelaxationLengthScale = 1.4f;
	return Props;
}
