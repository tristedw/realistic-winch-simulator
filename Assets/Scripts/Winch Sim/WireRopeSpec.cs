using UnityEngine;

/// <summary>
/// Physical properties of a real wire rope / synthetic winch line.
/// All values SI. Defaults match 3/8" (9.5mm) 6x25 IWRC galvanized aircraft
/// cable, the standard line on a 9,500-12,000 lb class recovery winch.
/// </summary>
[CreateAssetMenu(fileName = "WireRopeSpec", menuName = "Winch Sim/Wire Rope Spec")]
public class WireRopeSpec : ScriptableObject
{
    public enum RopeType { SteelWireRope, SyntheticUHMWPE }

    [Header("Identity")]
    public RopeType type = RopeType.SteelWireRope;

    [Header("Geometry")]
    [Tooltip("Nominal rope diameter (m). 3/8\" = 0.00953, 5/16\" = 0.00794.")]
    public float diameter = 0.00953f;

    [Tooltip("Effective diameter multiplier when spooled. Wire rope nests between " +
             "wraps on the layer below, so layer pitch is ~0.87d, not 1.0d.")]
    [Range(0.75f, 1.0f)] public float layerNestFactor = 0.87f;

    [Header("Mass")]
    [Tooltip("Linear mass density (kg/m). 3/8\" 6x25 IWRC = 0.36 kg/m.")]
    public float linearDensity = 0.36f;

    [Header("Strength")]
    [Tooltip("Minimum breaking strength (N). 3/8\" 6x25 IWRC = 65,400 N (14,700 lbf).")]
    public float minimumBreakingStrength = 65400f;

    [Tooltip("Design factor. Rated working load = MBS / designFactor. " +
             "Recovery winching typically runs 2-3; rigging standards use 5.")]
    [Range(1f, 10f)] public float designFactor = 2f;

    [Header("Elasticity")]
    [Tooltip("Effective elastic modulus of the ROPE, not the wire (Pa). " +
             "Stranded rope is far softer than solid steel (200 GPa) because " +
             "strands re-seat under load. 6x25 IWRC ~ 83 GPa. UHMWPE ~ 4 GPa.")]
    public float elasticModulus = 83e9f;

    [Tooltip("Metallic area factor: fraction of the nominal circle that is actual " +
             "metal. 6x25 IWRC ~ 0.46 (air gaps between strands).")]
    [Range(0.2f, 1f)] public float metallicAreaFactor = 0.46f;

    [Header("Bending")]
    [Tooltip("Minimum bend radius as a multiple of rope diameter. Below this the " +
             "rope is permanently damaged. Wire rope on a drum: ~18d.")]
    public float minBendRadiusFactor = 18f;

    [Tooltip("Internal friction of the rope resisting bending. Higher = the rope " +
             "holds coils and kinks; steel rope is stiff, synthetic is limp.")]
    [Range(0f, 1f)] public float bendStiffness = 0.35f;

    [Header("Damping")]
    [Tooltip("Structural damping ratio of the rope in axial vibration. " +
             "Steel rope 0.02-0.05, synthetic 0.08-0.15.")]
    [Range(0.005f, 0.4f)] public float dampingRatio = 0.04f;

    [Tooltip("Aerodynamic drag coefficient used for swing damping in air.")]
    [Range(0f, 2f)] public float airDragCoefficient = 1.1f;

    // ---- Derived ----

    /// <summary>Load-bearing cross-section (m^2).</summary>
    public float MetallicArea =>
        Mathf.PI * 0.25f * diameter * diameter * metallicAreaFactor;

    /// <summary>Axial stiffness EA (N). Force to stretch the rope 100% of its length.</summary>
    public float AxialStiffness => elasticModulus * MetallicArea;

    /// <summary>Rated working load (N).</summary>
    public float WorkingLoadLimit => minimumBreakingStrength / Mathf.Max(1f, designFactor);

    /// <summary>Minimum safe bend radius (m).</summary>
    public float MinBendRadius => minBendRadiusFactor * diameter;

    /// <summary>Spacing between adjacent wraps on the drum (m).</summary>
    public float WrapPitch => diameter;

    /// <summary>Radial growth per completed layer on the drum (m).</summary>
    public float LayerHeight => diameter * layerNestFactor;

    /// <summary>Spring constant of a free length of rope (N/m).</summary>
    public float StiffnessForLength(float length) =>
        AxialStiffness / Mathf.Max(1e-4f, length);

    /// <summary>Mass of a free length of rope (kg).</summary>
    public float MassForLength(float length) => linearDensity * length;

    /// <summary>
    /// Critical damping coefficient for a segment, so damping is expressed as a
    /// physical ratio instead of a magic number. c = 2 * zeta * sqrt(k * m).
    /// </summary>
    public float DampingForSegment(float segmentLength, float segmentMass)
    {
        float k = StiffnessForLength(segmentLength);
        return 2f * dampingRatio * Mathf.Sqrt(Mathf.Max(1e-6f, k * segmentMass));
    }

    /// <summary>Applies a named real-world preset.</summary>
    public void ApplyPreset(Preset p)
    {
        switch (p)
        {
            case Preset.Steel_5_16: // 5/16" 6x25 IWRC
                type = RopeType.SteelWireRope;
                diameter = 0.00794f; linearDensity = 0.25f;
                minimumBreakingStrength = 45400f;
                elasticModulus = 83e9f; metallicAreaFactor = 0.46f;
                minBendRadiusFactor = 18f; bendStiffness = 0.35f;
                dampingRatio = 0.04f; layerNestFactor = 0.87f;
                break;

            case Preset.Steel_3_8: // 3/8" 6x25 IWRC  -- default
                type = RopeType.SteelWireRope;
                diameter = 0.00953f; linearDensity = 0.36f;
                minimumBreakingStrength = 65400f;
                elasticModulus = 83e9f; metallicAreaFactor = 0.46f;
                minBendRadiusFactor = 18f; bendStiffness = 0.35f;
                dampingRatio = 0.04f; layerNestFactor = 0.87f;
                break;

            case Preset.Synthetic_3_8: // 3/8" 12-strand UHMWPE
                type = RopeType.SyntheticUHMWPE;
                diameter = 0.00953f; linearDensity = 0.062f;
                minimumBreakingStrength = 78000f;
                elasticModulus = 4.0e9f; metallicAreaFactor = 0.90f;
                minBendRadiusFactor = 6f; bendStiffness = 0.08f;
                dampingRatio = 0.12f; layerNestFactor = 0.92f;
                break;
        }
    }

    public enum Preset { Steel_5_16, Steel_3_8, Synthetic_3_8 }
}
