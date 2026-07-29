using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Draws the rope actually wound on the drum.
///
/// The drivetrain already knows exactly how the rope is stacked: WinchDrum
/// walks the layer stack every time the spooled length changes, because the
/// effective radius is what sets line pull and line speed. This component draws
/// that same stack instead of inventing one, so what you see on the barrel is
/// the geometry the physics is using. Spool in and the wraps build up layer by
/// layer; spool out and they disappear from the top down.
///
/// Winding pattern is the real one: each layer fills flange to flange, and the
/// next layer starts where the last one finished and runs back the other way.
///
/// Put this on a child of the rotating drum visual. The wrap is fixed to the
/// drum, so the mesh is built once per length change and rotation is free.
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class WinchDrumRopeVisual : MonoBehaviour
{
    [Header("Source")]
    [Tooltip("Drum whose spool state is drawn. If null, searched in parents.")]
    public WinchDrum drum;

    [Tooltip("Rotating drum transform the wrap is fixed to. If null, this " +
             "object's own transform is used, so parent it to the drum visual.")]
    public Transform drumBody;

    [Tooltip("Local spin axis of the drum. Must match WinchController.drumSpinAxis.")]
    public Vector3 spinAxis = Vector3.forward;

    [Header("Fit To Model")]
    [Tooltip("Radius the first layer is drawn at (m). 0 uses the drum's real " +
             "barrel radius.\n\n" +
             "Imported drum meshes are rarely modelled to the geometry the winch " +
             "is actually solving. The barrel in this scene is 80 mm where the " +
             "drum spec says 32 mm, so the entire rope stack would be drawn " +
             "buried inside the model and never seen. This lifts the stack onto " +
             "the surface that is visible. Layer height and wrap pitch stay real " +
             "and every layer still holds the rope length the physics says it " +
             "does, so the wrap grows and shrinks at the correct rate; only the " +
             "radius it starts from moves.")]
    public float visualBarrelRadius = 0f;

    [Tooltip("Width the wraps are spread over (m). 0 uses the drum's real usable " +
             "width. Raise it only if the model's flanges are further apart than " +
             "the simulated drum, and expect visible gaps between wraps, because " +
             "the rope is still drawn at its real diameter.")]
    public float visualDrumWidth = 0f;

    [Header("Tube")]
    [Range(3, 16)] public int radialSegments = 6;

    [Tooltip("Points generated per wrap. 12 is round enough at drum scale.")]
    [Range(4, 48)] public int pointsPerWrap = 12;

    [Tooltip("Ceiling on generated wraps. A full 26 m drum is about 115 wraps; " +
             "this only exists so a mis-set rope spec cannot build a million " +
             "vertex mesh.")]
    public int maxWraps = 400;

    [Tooltip("Material for the wrap. Leave null to keep whatever is on the " +
             "MeshRenderer already.")]
    public Material ropeMaterial;

    [Header("Rope Exit")]
    [Tooltip("Move the drum's rope exit point to where the rope actually leaves " +
             "the current layer.\n\n" +
             "The rope departs the drum at the tangent point from the fairlead " +
             "to the wrap surface, and that surface grows by a rope diameter " +
             "every layer. With a fixed exit transform the free rope is drawn " +
             "sprouting from the middle of the barrel and the visible wrap, " +
             "which is the most obvious tell that the two are unrelated.")]
    public bool driveExitPoint = true;

    // ---- Internals ----
    private Mesh _mesh;
    private MeshFilter _filter;
    private MeshRenderer _renderer;
    private readonly List<Vector3> _points = new List<Vector3>();

    private float _builtLength = float.NaN;
    private float _builtDiameter = float.NaN;
    private float _builtShift = float.NaN;
    private float _builtWidth = float.NaN;

    /// <summary>Point where the last wrap ends, in world space.</summary>
    public Vector3 WrapEnd { get; private set; }

    /// <summary>How far the drawn stack is lifted off the simulated one (m).</summary>
    public float RadiusShift => visualBarrelRadius > 0f && drum != null
        ? visualBarrelRadius - drum.barrelRadius : 0f;

    /// <summary>Radius the rope is drawn leaving the drum at (m).</summary>
    public float VisualEffectiveRadius =>
        drum != null ? drum.EffectiveRadius + RadiusShift : 0f;

    void Awake()
    {
        _filter = GetComponent<MeshFilter>();
        _renderer = GetComponent<MeshRenderer>();

        _mesh = new Mesh { name = "DrumRopeMesh" };
        _mesh.MarkDynamic();
        _filter.mesh = _mesh;

        if (ropeMaterial != null) _renderer.sharedMaterial = ropeMaterial;
        if (drum == null) drum = GetComponentInParent<WinchDrum>();
        if (drumBody == null) drumBody = transform;
    }

    void LateUpdate()
    {
        if (drum == null || drum.rope == null) return;

        float spooled = drum.SpooledLength;
        float d = drum.rope.diameter;

        // Only rebuild when the free end has moved far enough to change a point.
        // Everything else about the wrap is fixed to the drum and rides its
        // rotation for free.
        float step = 2f * Mathf.PI * drum.EffectiveRadius / Mathf.Max(4, pointsPerWrap);
        if (!Mathf.Approximately(_builtShift, RadiusShift) ||
            !Mathf.Approximately(_builtWidth, visualDrumWidth)) _builtLength = float.NaN;
        bool stale = float.IsNaN(_builtLength) ||
                     !Mathf.Approximately(d, _builtDiameter) ||
                     Mathf.Abs(spooled - _builtLength) > step;

        if (stale)
        {
            BuildWrap(spooled);
            _builtLength = spooled;
            _builtDiameter = d;
            _builtShift = RadiusShift;
            _builtWidth = visualDrumWidth;
        }

        if (driveExitPoint) PlaceExitPoint();
    }

    /// <summary>
    /// Generates the helix for <paramref name="spooled"/> metres of rope.
    ///
    /// Points are built in world space from the drum's own basis vectors, not in
    /// its local space. Drum meshes are almost always imported at some arbitrary
    /// scale (this scene's is at 5.25), and every dimension the drum reasons
    /// about -- barrel radius, rope diameter, layer height -- is in real metres.
    /// Building in local units would draw the wrap at whatever scale the artist
    /// happened to export at. Taking the basis vectors normalised and the centre
    /// from the transform keeps the geometry in metres while still riding the
    /// drum's rotation, and RopeTubeMesh converts back into mesh space at the end.
    /// </summary>
    void BuildWrap(float spooled)
    {
        _points.Clear();

        var rope = drum.rope;

        // The reference direction has to be fixed to the DRUM, not to the world.
        // Anchored to the world it would be recomputed at every rebuild while the
        // drum has turned underneath it, and the whole wrap would snap back to
        // the world reference each time.
        Vector3 axisLocal = spinAxis.sqrMagnitude < 1e-6f ? Vector3.forward : spinAxis.normalized;
        Vector3 uLocal = Vector3.Cross(axisLocal, Vector3.up);
        if (uLocal.sqrMagnitude < 1e-5f) uLocal = Vector3.Cross(axisLocal, Vector3.right);
        uLocal.Normalize();
        Vector3 vLocal = Vector3.Cross(axisLocal, uLocal);

        Vector3 centre = drumBody.position;
        Vector3 axis = drumBody.TransformDirection(axisLocal).normalized;
        Vector3 u = drumBody.TransformDirection(uLocal).normalized;
        Vector3 v = drumBody.TransformDirection(vLocal).normalized;

        int perLayer = drum.WrapsPerLayer;
        float width = visualDrumWidth > 0f ? visualDrumWidth : drum.drumWidth;
        float pitch = width / Mathf.Max(1, perLayer);
        float halfWidth = width * 0.5f;
        float shift = RadiusShift;

        float remaining = Mathf.Max(0f, spooled);
        float theta = 0f;
        int layer = 0;
        int wrapsDrawn = 0;

        while (remaining > 1e-4f && layer < 64 && wrapsDrawn < maxWraps)
        {
            // Rope is consumed at the real radius and drawn at the shifted one,
            // so moving the wrap onto the model cannot change how much rope a
            // layer holds.
            float rTrue = drum.RadiusAtLayer(layer);
            float r = rTrue + shift;
            float circumference = 2f * Mathf.PI * rTrue;
            bool forward = (layer & 1) == 0;   // alternate direction each layer

            for (int k = 0; k < perLayer && remaining > 1e-4f && wrapsDrawn < maxWraps; k++)
            {
                int slot = forward ? k : perLayer - 1 - k;
                float z = -halfWidth + pitch * (slot + 0.5f);

                // A partial wrap when the rope runs out mid layer.
                float frac = Mathf.Clamp01(remaining / circumference);
                int steps = Mathf.Max(1, Mathf.CeilToInt(pointsPerWrap * frac));

                for (int s = 0; s <= steps; s++)
                {
                    // Skip the duplicate joining point between consecutive wraps.
                    if (s == 0 && _points.Count > 0) continue;

                    float t = theta + (s / (float)steps) * frac * Mathf.PI * 2f;
                    _points.Add(centre +
                                u * (Mathf.Cos(t) * r) +
                                v * (Mathf.Sin(t) * r) +
                                axis * z);
                }

                theta += frac * Mathf.PI * 2f;
                remaining -= circumference * frac;
                wrapsDrawn++;
            }

            layer++;
        }

        if (_points.Count < 2) { _mesh.Clear(); return; }

        WrapEnd = _points[_points.Count - 1];

        RopeTubeMesh.Build(_mesh, _points, _points.Count,
                           rope.diameter * 0.5f, radialSegments, transform);
    }

    /// <summary>
    /// Puts the drum's rope exit at the tangent point from the fairlead to the
    /// current wrap surface. That is where a rope physically leaves a drum: it
    /// lies on the barrel until the line to the fairlead stops touching it.
    /// </summary>
    void PlaceExitPoint()
    {
        var exit = drum.ropeExitPoint;
        if (exit == null) return;

        Vector3 axis = drumBody.TransformDirection(
            spinAxis.sqrMagnitude < 1e-6f ? Vector3.forward : spinAxis).normalized;

        Vector3 centre = drumBody.position;
        float r = VisualEffectiveRadius;

        // Keep whatever axial position the exit was authored at. Sliding it to
        // the drum's centre line would drag the free rope sideways every time
        // the wrap radius changed.
        Vector3 offset = exit.position - centre;
        float axial = Vector3.Dot(offset, axis);

        Vector3 current = offset - axis * axial;
        if (current.sqrMagnitude < 1e-8f) return;

        // Without a fairlead there is nothing to be tangent to, so the exit just
        // rides the wrap surface in the direction it already faces.
        if (drum.fairlead == null)
        {
            exit.position = centre + current.normalized * r + axis * axial;
            return;
        }

        Vector3 planar = Vector3.ProjectOnPlane(drum.fairlead.position - centre, axis);
        if (planar.sqrMagnitude < 1e-8f) return;

        float dist = planar.magnitude;
        planar /= dist;

        if (dist <= r * 1.001f)
        {
            // Fairlead sits inside the wrap radius, so there is no tangent.
            exit.position = centre + planar * r + axis * axial;
            return;
        }

        // Tangent point: swing the centre-to-fairlead direction back by
        // acos(r / dist), the angle the tangent subtends at the drum centre.
        float angle = Mathf.Acos(Mathf.Clamp01(r / dist)) * Mathf.Rad2Deg;

        // Two tangents exist; the rope uses the one on the side it already
        // leaves from, which is the side the wrap was wound onto.
        Vector3 side = Vector3.Cross(axis, planar);
        float sign = Vector3.Dot(current, side) >= 0f ? 1f : -1f;

        Vector3 dir = Quaternion.AngleAxis(sign * angle, axis) * planar;
        exit.position = centre + dir * r + axis * axial;
    }

    void OnValidate()
    {
        maxWraps = Mathf.Max(1, maxWraps);
        if (spinAxis.sqrMagnitude < 1e-6f) spinAxis = Vector3.forward;
    }
}
