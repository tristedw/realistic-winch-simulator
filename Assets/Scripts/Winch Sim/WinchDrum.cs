using UnityEngine;

/// <summary>
/// Geometry and spooling state of a winch drum.
///
/// This is the component that makes a winch behave like a winch rather than a
/// constant-force puller: rope wraps onto the barrel in layers, each completed
/// layer increases the effective radius, and line pull is inversely proportional
/// to that radius while line speed is proportional to it. A 12,000 lb winch only
/// pulls 12,000 lb on the FIRST layer -- by the outer layer it is down to roughly
/// half that. Every real spec sheet is published as a per-layer table for exactly
/// this reason, and reproducing it is the core of an accurate winch sim.
/// </summary>
public class WinchDrum : MonoBehaviour
{
    [Header("Rope")]
    public WireRopeSpec rope;

    [Header("Drum Geometry")]
    [Tooltip("Bare barrel radius (m). Typical 9.5k-12k class winch: 0.0318 m (2.5\" dia).")]
    public float barrelRadius = 0.0318f;

    [Tooltip("Usable width between flanges (m). Typical: 0.2286 m (9\").")]
    public float drumWidth = 0.2286f;

    [Tooltip("Total rope carried on the drum (m). Typical: 25.9 m (85 ft).")]
    public float totalRopeLength = 25.9f;

    [Tooltip("Wraps that must stay on the drum at all times. Real winch manuals " +
             "require a minimum of 5 -- the drum anchor is not rated to hold load.")]
    public int minimumWrapsOnDrum = 5;

    [Header("Inertia")]
    [Tooltip("Rotational inertia of the bare drum + spool shaft (kg*m^2).")]
    public float bareDrumInertia = 0.006f;

    [Header("Rope Exit")]
    [Tooltip("Point where rope leaves the drum toward the fairlead. " +
             "If null, uses this transform.")]
    public Transform ropeExitPoint;

    [Tooltip("Fairlead / roller the rope passes through on its way out.")]
    public Transform fairlead;

    // ---- Spooling state ----

    /// <summary>Rope currently paid out past the drum (m).</summary>
    public float DeployedLength { get; private set; }

    /// <summary>Rope still wound on the drum (m).</summary>
    public float SpooledLength => Mathf.Max(0f, totalRopeLength - DeployedLength);

    /// <summary>Total drum revolutions since start (signed; + = spool in).</summary>
    public float Revolutions { get; private set; }

    // ---- Derived geometry ----

    /// <summary>Number of wraps that fit side by side across one layer.</summary>
    public int WrapsPerLayer =>
        rope == null ? 1 : Mathf.Max(1, Mathf.FloorToInt(drumWidth / rope.WrapPitch));

    /// <summary>Rope length stored in the layer whose index is <paramref name="layer"/> (0-based).</summary>
    public float LengthOfLayer(int layer)
    {
        if (rope == null) return 0f;
        float r = RadiusAtLayer(layer);
        return WrapsPerLayer * 2f * Mathf.PI * r;
    }

    /// <summary>Rope centreline radius while winding layer <paramref name="layer"/> (0-based).</summary>
    public float RadiusAtLayer(int layer)
    {
        if (rope == null) return barrelRadius;
        // Centre of the rope sits half a diameter above the surface it lies on.
        return barrelRadius + rope.diameter * 0.5f + layer * rope.LayerHeight;
    }

    /// <summary>Maximum layers that physically fit given rope capacity.</summary>
    public int LayerCount
    {
        get
        {
            if (rope == null) return 1;
            float remaining = totalRopeLength;
            int layers = 0;
            while (remaining > 0f && layers < 64)
            {
                remaining -= LengthOfLayer(layers);
                layers++;
            }
            return Mathf.Max(1, layers);
        }
    }

    // ---- Cached spool geometry ----
    //
    // Layer, radius and inertia all mean walking the layer stack, and the
    // drivetrain reads them several times per physics step (torque in, speed
    // out, inertia for the integrator, telemetry on top). They only change when
    // the deployed length does, so walk once and cache. Same numbers, one pass.

    private bool _geomValid;
    private int _cLayer;
    private float _cRadius;
    private float _cInertia;

    void InvalidateGeometry() => _geomValid = false;

    void RebuildGeometry()
    {
        _geomValid = true;

        if (rope == null)
        {
            _cLayer = 0;
            _cRadius = barrelRadius;
            _cInertia = bareDrumInertia;
            return;
        }

        float remaining = SpooledLength;
        float inertia = bareDrumInertia;
        int layer = 0;

        // Walk full layers, accumulating rope inertia as thin rings (I = m r^2).
        while (layer < 63)
        {
            float cap = LengthOfLayer(layer);
            if (remaining <= cap) break;
            float r = RadiusAtLayer(layer);
            inertia += rope.MassForLength(cap) * r * r;
            remaining -= cap;
            layer++;
        }

        float current = LengthOfLayer(layer);
        float frac = current > 1e-5f ? Mathf.Clamp01(remaining / current) : 0f;

        float rPartial = RadiusAtLayer(layer);
        inertia += rope.MassForLength(Mathf.Max(0f, remaining)) * rPartial * rPartial;

        _cLayer = layer;
        // Blend toward the next layer's radius as this one fills, so pull and
        // speed change smoothly instead of stepping at layer boundaries.
        _cRadius = Mathf.Lerp(rPartial, RadiusAtLayer(layer + 1), frac);
        _cInertia = inertia;
    }

    /// <summary>
    /// Current layer index the rope is winding on, from the rope actually spooled.
    /// This is the value the whole drivetrain hangs off.
    /// </summary>
    public int CurrentLayer
    {
        get { if (!_geomValid) RebuildGeometry(); return _cLayer; }
    }

    /// <summary>
    /// Effective rope radius right now (m). Interpolated within the layer so
    /// pull/speed change smoothly instead of stepping at layer boundaries.
    /// </summary>
    public float EffectiveRadius
    {
        get { if (!_geomValid) RebuildGeometry(); return _cRadius; }
    }

    /// <summary>Rope mass currently on the drum plus the barrel's own inertia (kg*m^2).</summary>
    public float TotalInertia
    {
        get { if (!_geomValid) RebuildGeometry(); return _cInertia; }
    }

    /// <summary>Fraction of rope paid out, 0 = fully spooled, 1 = fully deployed.</summary>
    public float PayoutFraction => totalRopeLength > 0f
        ? Mathf.Clamp01(DeployedLength / totalRopeLength) : 0f;

    /// <summary>Rope left on the drum expressed in wraps.</summary>
    public float WrapsRemaining
    {
        get
        {
            float r = EffectiveRadius;
            float circumference = 2f * Mathf.PI * Mathf.Max(1e-4f, r);
            return SpooledLength / circumference;
        }
    }

    public bool AtMinimumWraps => WrapsRemaining <= minimumWrapsOnDrum;
    public bool FullySpooled => DeployedLength <= 1e-4f;

    /// <summary>True if the rope is bent tighter than it should be on this drum.</summary>
    public bool ViolatesMinBendRadius =>
        rope != null && barrelRadius < rope.MinBendRadius;

    public Vector3 ExitPosition =>
        ropeExitPoint != null ? ropeExitPoint.position : transform.position;

    // ---- Mutation ----

    /// <summary>
    /// Advance the spool by a drum rotation of <paramref name="deltaAngleRad"/>.
    /// Positive winds rope IN. Returns the rope length actually moved (m),
    /// which is less than requested when the drum hits a hard limit.
    /// </summary>
    public float Rotate(float deltaAngleRad)
    {
        float r = EffectiveRadius;
        float requested = deltaAngleRad * r;   // arc length at the rope surface

        float before = DeployedLength;
        float maxPayout = totalRopeLength - MinimumRopeOnDrum();
        DeployedLength = Mathf.Clamp(DeployedLength - requested, 0f, maxPayout);
        float moved = before - DeployedLength;

        InvalidateGeometry();

        Revolutions += deltaAngleRad / (2f * Mathf.PI);
        return moved;
    }

    /// <summary>Rope that must never leave the drum (the minimum-wrap reserve), in m.</summary>
    public float MinimumRopeOnDrum()
    {
        float circumference = 2f * Mathf.PI * (barrelRadius + (rope != null ? rope.diameter * 0.5f : 0f));
        return minimumWrapsOnDrum * circumference;
    }

    /// <summary>Sets deployed rope directly, e.g. when freespooling by hand.</summary>
    public void SetDeployedLength(float meters)
    {
        DeployedLength = Mathf.Clamp(meters, 0f, totalRopeLength - MinimumRopeOnDrum());
        InvalidateGeometry();
    }

    /// <summary>Line speed (m/s) for a given drum angular velocity (rad/s).</summary>
    public float LineSpeed(float omega) => omega * EffectiveRadius;

    /// <summary>Line pull (N) produced by a given drum torque (N*m).</summary>
    public float LinePull(float drumTorque) => drumTorque / Mathf.Max(1e-4f, EffectiveRadius);

    /// <summary>Drum torque (N*m) required to hold a given rope tension (N).</summary>
    public float TorqueFromTension(float tension) => tension * EffectiveRadius;

    void OnValidate()
    {
        barrelRadius = Mathf.Max(0.005f, barrelRadius);
        drumWidth = Mathf.Max(0.01f, drumWidth);
        totalRopeLength = Mathf.Max(0.1f, totalRopeLength);
        minimumWrapsOnDrum = Mathf.Max(0, minimumWrapsOnDrum);
        InvalidateGeometry();   // barrel, width and rope all move the layer stack
    }

    void OnDrawGizmosSelected()
    {
        if (rope == null) return;

        Vector3 axis = transform.forward;
        Vector3 c = transform.position;

        // Barrel
        Gizmos.color = new Color(0.5f, 0.5f, 0.55f, 0.9f);
        DrawCircle(c - axis * drumWidth * 0.5f, axis, barrelRadius);
        DrawCircle(c + axis * drumWidth * 0.5f, axis, barrelRadius);

        // Full-capacity outer radius
        Gizmos.color = new Color(1f, 0.5f, 0.1f, 0.5f);
        float outer = RadiusAtLayer(LayerCount - 1) + rope.diameter * 0.5f;
        DrawCircle(c - axis * drumWidth * 0.5f, axis, outer);
        DrawCircle(c + axis * drumWidth * 0.5f, axis, outer);

        // Live rope surface
        if (Application.isPlaying)
        {
            Gizmos.color = Color.cyan;
            DrawCircle(c, axis, EffectiveRadius);
        }
    }

    static void DrawCircle(Vector3 center, Vector3 axis, float radius, int steps = 48)
    {
        Vector3 a = Vector3.Cross(axis, Vector3.up);
        if (a.sqrMagnitude < 1e-5f) a = Vector3.Cross(axis, Vector3.right);
        a = a.normalized * radius;
        Vector3 b = Vector3.Cross(axis.normalized, a);

        Vector3 prev = center + a;
        for (int i = 1; i <= steps; i++)
        {
            float t = i / (float)steps * Mathf.PI * 2f;
            Vector3 p = center + a * Mathf.Cos(t) + b * Mathf.Sin(t);
            Gizmos.DrawLine(prev, p);
            prev = p;
        }
    }
}
