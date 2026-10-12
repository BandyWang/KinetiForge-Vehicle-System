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

	// for materials that are always snowy (ski areas, high Sierra roads in winter)
	FVehicleSurfaceEntry Snow;
	Snow.Properties = MakeSnow();
	Surfaces.Add(Snow);

	SnowSurface = MakeSnow();
}

TSharedRef<const FVehicleSurfaceLookup, ESPMode::ThreadSafe> UVehicleSurfaceDataAsset::BuildLookup(const TMap<FName, float>* TireGripMultipliers) const
{
	TSharedRef<FVehicleSurfaceLookup, ESPMode::ThreadSafe> Lookup = MakeShared<FVehicleSurfaceLookup, ESPMode::ThreadSafe>();

	Lookup->bIgnorePhysicalMaterialFriction = bIgnorePhysicalMaterialFriction;
	Lookup->DefaultSurface = DefaultSurface;

	auto ApplyTireMultiplier = [TireGripMultipliers](FVehicleSurfaceProperties& Props)
	{
		if (TireGripMultipliers)
		{
			if (const float* Multiplier = TireGripMultipliers->Find(Props.SurfaceName))
			{
				Props.GripScale *= FMath::Max(0.f, *Multiplier);
			}
		}
	};

	Lookup->SnowSurface = SnowSurface;

	ApplyTireMultiplier(Lookup->DefaultSurface);
	ApplyTireMultiplier(Lookup->SnowSurface);

	for (const FVehicleSurfaceEntry& Entry : Surfaces)
	{
		const int32 SurfaceIndex = Lookup->Surfaces.Add(Entry.Properties);
		ApplyTireMultiplier(Lookup->Surfaces[SurfaceIndex]);

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
	Props.WetGripScale = 0.7f;
	Props.WetRollingResistanceScale = 1.2f;
	Props.IceGripScale = 0.15f;
	return Props;
}

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::MakeWetAsphalt()
{
	FVehicleSurfaceProperties Props;
	Props.SurfaceName = TEXT("WetAsphalt");
	// for materials that are always wet (puddles, creek crossings); rain on dry asphalt uses WetGripScale instead
	// wet mu ~0.5-0.8; no standing water (aquaplaning is not modeled)
	Props.GripScale = 0.7f;
	Props.LongitudinalGripScale = 1.f;
	Props.LateralGripScale = 1.f;
	Props.RollingResistanceCoefficient = 0.015f;
	Props.RelaxationLengthScale = 1.f;
	Props.WetGripScale = 0.85f;
	Props.WetRollingResistanceScale = 1.f;
	Props.IceGripScale = 0.2f;
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
	Props.WetGripScale = 0.85f;
	Props.WetRollingResistanceScale = 1.1f;
	Props.IceGripScale = 0.3f;
	return Props;
}

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::MakeDirt()
{
	FVehicleSurfaceProperties Props;
	Props.SurfaceName = TEXT("Dirt");
	// dry packed dirt; rain turns it to mud through WetGripScale / WetRollingResistanceScale
	Props.GripScale = 0.65f;
	Props.LongitudinalGripScale = 1.f;
	Props.LateralGripScale = 0.95f;
	Props.RollingResistanceCoefficient = 0.04f;
	Props.RelaxationLengthScale = 1.4f;
	Props.WetGripScale = 0.6f;
	Props.WetRollingResistanceScale = 1.6f;
	Props.IceGripScale = 0.25f;
	return Props;
}

FVehicleSurfaceProperties UVehicleSurfaceDataAsset::MakeSnow()
{
	FVehicleSurfaceProperties Props;
	Props.SurfaceName = TEXT("Snow");
	// packed snow, mu ~0.2-0.35 for an all-season tire; fresh deep snow has much higher rolling resistance
	Props.GripScale = 0.3f;
	Props.LongitudinalGripScale = 1.f;
	Props.LateralGripScale = 1.f;
	Props.RollingResistanceCoefficient = 0.05f;
	Props.RelaxationLengthScale = 1.8f;
	// wet snow above freezing is slush; wet snow at or below freezing is glazed and icy
	Props.WetGripScale = 0.8f;
	Props.WetRollingResistanceScale = 1.5f;
	Props.IceGripScale = 0.5f;
	return Props;
}
