using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Wire rope simulated as a chain of rigid bodies joined by ConfigurableJoints.
///
/// Nothing here is a tuned magic number: node mass, damping and segment length
/// all come from a <see cref="WireRopeSpec"/> using real material physics.
///
///   segment mass       m = linearDensity * segmentLength
///   segment stiffness  k = EA / segmentLength      (Hooke's law for a rod)
///   segment damping    c = 2 * zeta * sqrt(k * m)  (structural damping ratio)
///   air drag           F = 0.5 * rho * Cd * A * v^2
///
/// So a 3/8" steel rope weighs what it weighs, sags into the catenary it should,
/// and settles at its real rate, with no hand tuning.
///
/// The rope supports continuous length change at runtime via
/// <see cref="SetLength"/> so a winch drum can spool it in and out. Nodes are
/// added and removed at the drum end only, so the far end of the rope keeps its
/// position, velocity and contacts across a spool.
///
/// DIVISION OF LABOUR
/// By default (<see cref="loadCarryingEnds"/> off) this chain provides SHAPE:
/// sag, swing, collision and the visible line. It does not carry the load. The
/// load path is solved analytically by <see cref="WinchRopeTether"/>, because a
/// serial chain of ~100 g links pulling a 1,200 kg vehicle is a 12,000:1 mass
/// ratio that no iterative solver handles -- measured here, it either stretched
/// to twice its length while the drum read no tension, or pumped energy until
/// the load left at 180 m/s. Splitting the two jobs gives an accurate load path
/// and a rope that looks right, instead of one component doing both badly.
/// </summary>
[RequireComponent(typeof(CableRenderer))]
public class CableSimulator : MonoBehaviour
{
    [Header("Rope")]
    [Tooltip("Physical rope properties. If null, a 3/8\" steel default is used.")]
    public WireRopeSpec rope;

    [Header("Discretisation")]
    [Tooltip("Target physical length of one simulated segment, as a multiple of " +
             "rope diameter. Lower = more accurate bending, more cost. " +
             "20-40d is the useful band.")]
    [Range(5f, 100f)] public float segmentLengthInDiameters = 28f;

    [Tooltip("Hard bounds on node count so spooling cannot explode the sim.")]
    public int minNodes = 4;
    public int maxNodes = 64;

    [Header("Rope Length")]
    [Tooltip("Free rope length between the two ends (m). Driven by the winch " +
             "drum at runtime.")]
    public float cableLength = 2f;

    [Header("Solver")]
    [Tooltip("Per-body solver iterations for rope nodes. A rope is a long serial " +
             "chain, so force has to propagate link by link; the project-wide " +
             "default is nowhere near enough and the rope visibly tears apart " +
             "under load without this.")]
    [Range(6, 128)] public int nodeSolverIterations = 40;

    [Range(4, 64)] public int nodeSolverVelocityIterations = 20;

    [Tooltip("Hard-correct joints that the solver could not satisfy. Without " +
             "this a loaded rope drifts several times past its stretch limit.")]
    public bool useJointProjection = true;

    [Tooltip("Largest mass ratio the solver is asked to handle at a rope-to-load " +
             "joint. A 0.1 kg rope node against a 1,200 kg vehicle is a ratio of " +
             "12,000:1, which PhysX simply cannot solve -- the constraint goes " +
             "soft and the rope stretches without ever building tension. Joint " +
             "mass scaling tells the solver to treat the pair as this ratio " +
             "instead; the true masses are untouched, so momentum and the forces " +
             "reported back to the winch stay correct.")]
    [Range(1f, 1000f)] public float maxSolverMassRatio = 60f;

    [Tooltip("How far the segment length must drift before the joints are " +
             "rewritten, as a fraction. Rewriting a joint resets its solver " +
             "impulse, so doing it every step while spooling stops the rope " +
             "ever building tension. Keep this well above zero.")]
    [Range(0.002f, 0.2f)] public float jointPatchThreshold = 0.03f;

    [Tooltip("Fraction of the theoretical EA stiffness actually handed to PhysX. " +
             "Real steel rope stiffness (~5 MN) is beyond what any game solver " +
             "can integrate stably, so the rope is made compliant here and the " +
             "hard length cap below carries the load instead. 1e-4 to 1e-2 is " +
             "the stable band; the rope still behaves as inextensible because " +
             "the joint limit stops it.")]
    [Range(1e-5f, 0.1f)] public float stiffnessScale = 3e-3f;

    [Tooltip("Hard maximum stretch per segment as a fraction of segment length. " +
             "This is what actually carries load. Steel rope constructional " +
             "stretch is 0.5-1%, so 0.005-0.01 is physically right.")]
    [Range(0f, 0.1f)] public float stretchLimit = 0.008f;

    [Tooltip("Stiffness of the stretch limit (N/m). LEAVE THIS AT 0: zero makes " +
             "PhysX treat the limit as a hard constraint solved exactly, which " +
             "is what an inextensible rope needs. A finite spring lets the rope " +
             "stretch by tension/spring, so under a 15 kN winch load even a " +
             "40,000 N/m limit would let every segment grow by 375 mm.")]
    public float limitSpring = 0f;

    public float limitDamper = 0f;
    [Range(0f, 0.05f)] public float limitContactDistance = 0.002f;

    [Header("Aerodynamics")]
    [Tooltip("Apply air drag derived from rope diameter and drag coefficient " +
             "instead of Unity's abstract linear damping.")]
    public bool useAerodynamicDrag = true;

    [Tooltip("Air density (kg/m^3).")]
    public float airDensity = 1.225f;

    [Tooltip("Baseline Rigidbody damping. Kept low so real drag dominates.")]
    [Range(0f, 2f)] public float baseLinearDamping = 0.05f;
    [Range(0f, 20f)] public float baseAngularDamping = 2f;

    [Tooltip("Hard ceiling on node speed (m/s). Nothing on a real winch line " +
             "moves faster than a snapped rope end (~50 m/s), so anything above " +
             "this is solver noise. Clamping it turns a potential explosion into " +
             "a brief wobble instead of letting one bad step poison the scene.")]
    [Range(1f, 200f)] public float maxNodeSpeed = 45f;

    [Header("Gravity")]
    [Range(-2f, 2f)] public float gravityScale = 1f;

    [Header("Tension Shape")]
    [Tooltip("Pull the visible chain onto the catenary its own tension implies.\n\n" +
             "A loaded rope is held straight by the load. Sag of a line under " +
             "tension is s = w*L^2/(8*T), so 4 m of 3/8\" rope (w = 3.5 N/m) at " +
             "9 kN sags 0.8 mm. The rigidbody chain here deliberately carries no " +
             "load (see loadCarryingEnds), so it never feels the tension that " +
             "would straighten it and hangs at its full self-weight sag of " +
             "~100 mm while the winch is at rated pull. That is the floppiness.\n\n" +
             "This computes the sag the measured tension actually allows and " +
             "blends the chain onto it, so the rope stiffens as it loads up and " +
             "goes back to free physics when it goes slack. The load path is " +
             "untouched: this only moves the nodes that draw the line.")]
    public bool tensionShaping = true;

    [Tooltip("Time constant for handing shape control between the physics chain " +
             "and the analytic curve (s). Without it the rope snaps dead straight " +
             "in a single frame the moment tension appears.")]
    [Range(0f, 0.5f)] public float shapeBlendTime = 0.06f;

    [Header("Visual")]
    [Tooltip("Drive the rendered tube radius from the rope spec instead of " +
             "whatever is set on CableMeshBuilder.")]
    public bool matchMeshToRopeDiameter = true;

    [Header("Failure")]
    [Tooltip("Break the rope when tension exceeds its minimum breaking strength.")]
    public bool simulateRopeFailure = true;

    [Tooltip("Extra margin on MBS before the rope actually parts.")]
    [Range(1f, 3f)] public float breakMargin = 1.0f;

    [Tooltip("Time constant for smoothing the tension readings (s). Iterative " +
             "solvers emit single-step force spikes that are numerical, not " +
             "physical; without smoothing they make the winch stutter and can " +
             "part a rope that was never actually overloaded.")]
    [Range(0f, 0.2f)] public float tensionSmoothing = 0.03f;

    [Tooltip("How long the rope must stay over its breaking strength before it " +
             "parts (s). Real rope does not fail on a one-millisecond spike.")]
    [Range(0f, 1f)] public float overloadDuration = 0.15f;

    [Header("Collision")]
    [Tooltip("Let the rope collide with itself. Off by default: adjacent segment " +
             "capsules touch end to end, so self-collision costs a lot and buys " +
             "very little for a line under tension.")]
    public bool selfCollision = false;

    public LayerMask collisionLayers = ~0;

    [Header("Anchors")]
    [Tooltip("If set, node 0 is pinned here. This is the drum / winch end.")]
    public Transform startAnchor;

    [Tooltip("If set, the last node is pinned here. This is the load end.")]
    public Transform endAnchor;

    [Tooltip("Let the rigidbody chain carry the load force between its ends.\n\n" +
             "OFF (default) is what you want with a winch. The chain then only " +
             "provides shape, sag, swing and collision, and the load path is " +
             "handled analytically by WinchRopeTether. A rope chain is a long " +
             "series of hard constraints between ~100 g links and a 1,200 kg " +
             "load -- a 12,000:1 mass ratio that no iterative solver can hold. " +
             "Turned ON here it measurably either stretches to twice its length " +
             "while the drum reads no tension, or launches the load at 180 m/s.\n\n" +
             "Turn it ON only for light loads where the chain can cope.")]
    public bool loadCarryingEnds = false;

    // ---- Public readouts ----

    /// <summary>Tension measured at the drum end of the rope (N).</summary>
    public float TensionAtStart { get; private set; }

    /// <summary>Tension measured at the load end of the rope (N).</summary>
    public float TensionAtEnd { get; private set; }

    /// <summary>Highest tension anywhere along the rope this frame (N).</summary>
    public float PeakTension { get; private set; }

    /// <summary>Tension as a fraction of the rope's breaking strength.</summary>
    public float LoadFraction => Spec.minimumBreakingStrength > 0f
        ? PeakTension / Spec.minimumBreakingStrength : 0f;

    /// <summary>True once the rope has parted.</summary>
    public bool HasFailed { get; private set; }

    /// <summary>
    /// Tension measured outside this component, by whatever is actually solving
    /// the load path (normally <see cref="WinchRopeTether"/>). In visual-only
    /// mode the chain carries no load, so this is the real answer and the
    /// chain's own joint forces are just the rope's self-weight.
    /// </summary>
    public void ReportExternalTension(float newtons)
    {
        _externalTension = Mathf.Max(0f, newtons);
        _hasExternalTension = true;
    }

    /// <summary>Straight-line distance between the two rope ends (m).</summary>
    public float SpanDistance
    {
        get
        {
            if (_nodes.Count < 2) return 0f;
            return Vector3.Distance(_nodes[0].transform.position,
                                    _nodes[_nodes.Count - 1].transform.position);
        }
    }

    /// <summary>Actual summed length of the simulated chain (m).</summary>
    public float MeasuredLength
    {
        get
        {
            float total = 0f;
            for (int i = 1; i < _nodes.Count; i++)
                total += Vector3.Distance(_nodes[i - 1].transform.position,
                                          _nodes[i].transform.position);
            return total;
        }
    }

    /// <summary>How taut the rope is: 1 = dead straight, 0 = fully slack.</summary>
    public float Tautness => cableLength > 1e-4f
        ? Mathf.Clamp01(SpanDistance / cableLength) : 0f;

    public bool IsSlack => Tautness < 0.995f;

    /// <summary>
    /// How much of the rope's shape is currently dictated by tension rather than
    /// by the free physics chain. 0 = hanging under its own weight, 1 = pulled
    /// onto the tension catenary.
    /// </summary>
    public float ShapeAuthority { get; private set; }

    /// <summary>Sag the current tension allows at the current span (m).</summary>
    public float TensionSag { get; private set; }

    public Rigidbody GetNode(int i) =>
        (i >= 0 && i < _rbs.Count) ? _rbs[i] : null;

    public int NodeCount => _nodes.Count;

    /// <summary>Number of simulated segments.</summary>
    public int SegmentCount => Mathf.Max(1, _nodes.Count - 1);

    /// <summary>Length of one simulated segment (m).</summary>
    public float SegmentLength => cableLength / Mathf.Max(1, _nodes.Count - 1);

    public WireRopeSpec Spec
    {
        get
        {
            if (rope != null) return rope;
            if (_fallbackSpec == null)
            {
                _fallbackSpec = ScriptableObject.CreateInstance<WireRopeSpec>();
                _fallbackSpec.ApplyPreset(WireRopeSpec.Preset.Steel_3_8);
            }
            return _fallbackSpec;
        }
    }

    // ---- Internals ----
    private readonly List<GameObject> _nodes = new List<GameObject>();
    private readonly List<Rigidbody> _rbs = new List<Rigidbody>();
    // _joints[i] is the joint living on node i+1, connecting it back to node i.
    private readonly List<ConfigurableJoint> _joints = new List<ConfigurableJoint>();
    // Joints coupling a rope end to a dynamic load, kept apart from the chain.
    private readonly List<ConfigurableJoint> _anchorJoints = new List<ConfigurableJoint>();
    private ConfigurableJoint _startJoint;   // rope -> drum. Reads drum tension.
    private ConfigurableJoint _endJoint;     // rope -> load.
    private CableRenderer _renderer;
    private WireRopeSpec _fallbackSpec;
    private Transform[] _rendererBuffer;

    private float _builtLength;
    private Transform _builtStartAnchor;
    private Transform _builtEndAnchor;
    private WireRopeSpec _builtSpec;
    private float _builtSegmentDiameters;
    private float _builtStiffnessScale;
    private float _builtStretchLimit;
    private bool _built;
    private float _overloadTimer;
    private float _patchedSegmentLength = -1f;
    private float _externalTension;
    private bool _hasExternalTension;

    void Awake() => _renderer = GetComponent<CableRenderer>();

    void Start() => Build();

    void OnDestroy() => Teardown();

    // ------------------------------------------------------------------
    // Length control
    // ------------------------------------------------------------------

    /// <summary>
    /// Sets the free rope length. Cheap enough to call every FixedUpdate;
    /// nodes are only added or removed when the segment count actually changes.
    /// </summary>
    public void SetLength(float meters)
    {
        meters = Mathf.Max(Spec.diameter * 2f, meters);
        if (_built && Mathf.Approximately(meters, cableLength))
        {
            // Still record it. Update() compares cableLength against _builtLength
            // to decide whether a length change needs applying, so leaving them
            // out of sync here made that test true forever and Update called
            // straight back into this early return every single frame.
            _builtLength = cableLength;
            return;
        }

        cableLength = meters;
        if (!_built) return;

        ResizeChain();

        // Only rewrite the joints when the segment length has actually moved.
        //
        // Assigning to a ConfigurableJoint's properties rebuilds the underlying
        // PhysX constraint and discards its accumulated impulse. Doing that
        // every FixedUpdate -- which is what happens if a spooling winch calls
        // SetLength each step -- means the chain restarts its solve from zero
        // every step and can never build tension: the rope reads a few newtons
        // at the drum while the load end reads tens of kilonewtons, and the
        // winch never feels its load.
        float segLen = SegmentLength;
        if (_patchedSegmentLength <= 0f ||
            Mathf.Abs(segLen - _patchedSegmentLength) > _patchedSegmentLength * jointPatchThreshold)
        {
            PatchSegmentGeometry();
            _patchedSegmentLength = segLen;
        }

        _builtLength = cableLength;
    }

    int DesiredNodeCount(float length)
    {
        float target = Spec.diameter * segmentLengthInDiameters;
        int segs = Mathf.Max(1, Mathf.RoundToInt(length / Mathf.Max(1e-4f, target)));
        return Mathf.Clamp(segs + 1, Mathf.Max(2, minNodes), Mathf.Max(minNodes, maxNodes));
    }

    /// <summary>
    /// Adds or removes nodes at the DRUM end (index 1) so the load end of the
    /// rope keeps its position, velocity and contacts across a length change.
    /// Hysteresis of one node stops thrash at the threshold.
    /// </summary>
    void ResizeChain()
    {
        int desired = DesiredNodeCount(cableLength);
        if (Mathf.Abs(desired - _nodes.Count) < 2) return;

        int guard = 0;
        while (_nodes.Count < desired && guard++ < 128)
            InsertNodeAtDrum();

        guard = 0;
        while (_nodes.Count > desired &&
               _nodes.Count > Mathf.Max(2, minNodes) && guard++ < 128)
            RemoveNodeAtDrum();

        RefreshRendererBuffer();
    }

    void InsertNodeAtDrum()
    {
        if (_nodes.Count < 2 || _joints.Count < 1) return;

        Vector3 spawn = _nodes[0].transform.position;
        Quaternion rot = _nodes[0].transform.rotation;

        GameObject go = CreateNode(_nodes.Count, spawn, rot, SegmentLength);
        Rigidbody rb = go.GetComponent<Rigidbody>();
        rb.linearVelocity = _rbs[1].linearVelocity;
        rb.angularVelocity = _rbs[1].angularVelocity;

        // Old chain: 0 <- 1 <- 2 ...   (_joints[0] lives on node 1)
        // New chain: 0 <- NEW <- 1 <- 2 ...
        var oldFirstJoint = _joints[0];

        _nodes.Insert(1, go);
        _rbs.Insert(1, rb);

        var j = CreateJoint(rb, _rbs[0], SegmentLength);
        _joints.Insert(0, j);

        if (oldFirstJoint != null) oldFirstJoint.connectedBody = rb;

        ApplySelfCollision();
        IgnoreAnchorCollisions();
        RenameNodes();
    }

    /// <summary>
    /// Stops the rope colliding with whatever it is attached to. A rope end
    /// terminates at the attachment point, which is usually inside the load's
    /// own collider, so without this the rope and the load fight a permanent
    /// interpenetration and report a large constant bogus force.
    /// </summary>
    void IgnoreAnchorCollisions()
    {
        IgnoreCollisionsWith(startAnchor);
        IgnoreCollisionsWith(endAnchor);

        foreach (var j in _anchorJoints)
            if (j != null && j.connectedBody != null)
                IgnoreCollisionsWith(j.connectedBody.transform);
    }

    void IgnoreCollisionsWith(Transform anchor)
    {
        if (anchor == null) return;

        // Walk up to the whole anchored object, not just the marker transform.
        Transform root = anchor;
        var rb = anchor.GetComponentInParent<Rigidbody>();
        if (rb != null) root = rb.transform;

        var theirs = root.GetComponentsInChildren<Collider>();
        if (theirs.Length == 0) return;

        for (int i = 0; i < _nodes.Count; i++)
        {
            var mine = _nodes[i] != null ? _nodes[i].GetComponent<Collider>() : null;
            if (mine == null) continue;
            foreach (var other in theirs)
                if (other != null) Physics.IgnoreCollision(mine, other, true);
        }
    }

    void RemoveNodeAtDrum()
    {
        if (_nodes.Count <= Mathf.Max(2, minNodes) || _joints.Count < 1) return;

        var doomedJoint = _joints[0];              // node1 -> node0
        if (_joints.Count > 1 && _joints[1] != null)
            _joints[1].connectedBody = _rbs[0];    // node2 -> node0

        if (doomedJoint != null) Destroy(doomedJoint);
        _joints.RemoveAt(0);

        Destroy(_nodes[1]);
        _nodes.RemoveAt(1);
        _rbs.RemoveAt(1);

        RenameNodes();
    }

    void RenameNodes()
    {
        for (int i = 0; i < _nodes.Count; i++)
            if (_nodes[i] != null) _nodes[i].name = $"RopeNode_{i}";
    }

    // ------------------------------------------------------------------
    // Build
    // ------------------------------------------------------------------

    void Build()
    {
        Teardown();

        int count = DesiredNodeCount(cableLength);
        float segLen = cableLength / Mathf.Max(1, count - 1);

        Vector3 startPos = startAnchor != null ? startAnchor.position : transform.position;
        Vector3 dir = endAnchor != null
            ? (endAnchor.position - startPos).normalized
            : -transform.up;
        if (dir.sqrMagnitude < 1e-6f) dir = -transform.up;

        Quaternion rot = Quaternion.LookRotation(dir, Vector3.up);

        for (int i = 0; i < count; i++)
        {
            var go = CreateNode(i, startPos + dir * segLen * i, rot, segLen);
            _nodes.Add(go);
            _rbs.Add(go.GetComponent<Rigidbody>());
        }

        for (int i = 1; i < count; i++)
            _joints.Add(CreateJoint(_rbs[i], _rbs[i - 1], segLen));

        _startJoint = startAnchor != null ? AttachEnd(_rbs[0], startAnchor) : null;
        _endJoint = endAnchor != null ? AttachEnd(_rbs[count - 1], endAnchor) : null;

        ApplySelfCollision();
        IgnoreAnchorCollisions();
        RefreshRendererBuffer();
        ApplyVisualRadius();
        PatchSegmentGeometry();   // now that the node count is known
        _patchedSegmentLength = SegmentLength;
        Snapshot();
        _built = true;
        HasFailed = false;
    }

    void Teardown()
    {
        foreach (var n in _nodes)
            if (n != null) Destroy(n);
        _nodes.Clear();
        _rbs.Clear();
        _joints.Clear();
        _anchorJoints.Clear();
        _startJoint = null;
        _endJoint = null;
        _overloadTimer = 0f;
        _built = false;
    }

    /// <summary>
    /// Creates one rope node. <paramref name="segLen"/> must be passed in rather
    /// than read from <see cref="SegmentLength"/>: during Build the node list is
    /// still filling up, so the derived property would return garbage and every
    /// node would get a different mass and collider size.
    /// </summary>
    GameObject CreateNode(int index, Vector3 pos, Quaternion rot, float segLen)
    {
        var go = new GameObject($"RopeNode_{index}");
        go.transform.SetParent(transform, true);
        go.transform.SetPositionAndRotation(pos, rot);
        go.layer = gameObject.layer;

        var rb = go.AddComponent<Rigidbody>();
        rb.mass = Mathf.Max(1e-4f, Spec.MassForLength(segLen));   // refined by PatchSegmentGeometry
        rb.linearDamping = baseLinearDamping;
        rb.angularDamping = baseAngularDamping;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.useGravity = true;
        rb.maxAngularVelocity = 60f;
        rb.maxDepenetrationVelocity = 2f;   // stops contact pops from launching the rope
        rb.solverIterations = nodeSolverIterations;
        rb.solverVelocityIterations = nodeSolverVelocityIterations;

        var col = go.AddComponent<CapsuleCollider>();
        col.radius = Spec.diameter * 0.5f;
        col.height = segLen + Spec.diameter;
        col.direction = 2;             // local Z, the chain axis
        col.center = new Vector3(0f, 0f, segLen * 0.5f);

        return go;
    }

    /// <summary>
    /// Turns off collisions between the rope and itself. Adjacent segment
    /// capsules necessarily touch end to end, so leaving self-collision on
    /// makes the rope buzz against itself for no physical benefit. Collisions
    /// with the world stay fully enabled.
    /// </summary>
    void ApplySelfCollision()
    {
        for (int i = 0; i < _nodes.Count; i++)
        {
            var a = _nodes[i] != null ? _nodes[i].GetComponent<Collider>() : null;
            if (a == null) continue;
            for (int k = i + 1; k < _nodes.Count; k++)
            {
                var b = _nodes[k] != null ? _nodes[k].GetComponent<Collider>() : null;
                if (b == null) continue;
                Physics.IgnoreCollision(a, b, !selfCollision);
            }
        }
    }

    ConfigurableJoint CreateJoint(Rigidbody child, Rigidbody parent, float segLen)
    {
        var j = child.gameObject.AddComponent<ConfigurableJoint>();
        j.connectedBody = parent;
        j.autoConfigureConnectedAnchor = false;
        j.anchor = Vector3.zero;
        j.connectedAnchor = Vector3.zero;
        j.enablePreprocessing = false;   // stiff chains behave far better without it
        ApplyJointPhysics(j, segLen);
        return j;
    }

    /// <summary>
    /// Writes rope-derived stiffness and damping into a joint.
    /// This is the heart of the realism: k = EA/L, c = 2*zeta*sqrt(k*m).
    /// </summary>
    void ApplyJointPhysics(ConfigurableJoint j, float segLen)
    {
        var spec = Spec;
        float segMass = Mathf.Max(1e-4f, spec.MassForLength(segLen));

        float kTrue = spec.StiffnessForLength(segLen);
        float k = kTrue * stiffnessScale;
        float c = 2f * spec.dampingRatio * Mathf.Sqrt(Mathf.Max(1e-6f, k * segMass));

        // ---- Axial behaviour ----
        //
        // A rope resists being pulled apart and does nothing at all when pushed
        // together, so the correct constraint is a pure MAXIMUM DISTANCE, not a
        // spring holding the nodes at a fixed separation.
        //
        // The earlier formulation drove each node to sit exactly at its distance
        // limit, which left every joint permanently pressed against its own
        // limit and fighting the solver. Here the limit sphere is the only
        // axial constraint: nodes are free anywhere inside it (slack rope coils
        // and sags naturally) and cannot leave it (taut rope is inextensible).
        //
        // Elongation is carried by the limit radius: stretchLimit is the rope's
        // constructional stretch, 0.5-1% for steel wire rope.
        j.xMotion = j.yMotion = j.zMotion = ConfigurableJointMotion.Limited;

        // contactDistance makes the solver start resolving the limit slightly
        // BEFORE it is reached. Without it a slack rope snapping taut resolves
        // in a single step as a hard slam, and projection then teleports the
        // bodies apart, which injects energy: a 1,200 kg load gets launched at
        // 16 m/s by a 1.4 kg rope. Engaging early turns that into a firm catch.
        j.linearLimit = new SoftJointLimit
        {
            limit = segLen * (1f + stretchLimit),
            contactDistance = Mathf.Max(limitContactDistance, segLen * 0.12f)
        };
        j.linearLimitSpring = new SoftJointLimitSpring
        {
            spring = limitSpring,     // 0 = solved as a hard constraint
            damper = limitDamper
        };

        // No positional drive. Its only job would be to fight the limit.
        var drive = new JointDrive
        {
            positionSpring = 0f,
            positionDamper = c * 0.05f,   // a little axial damping to kill ringing
            maximumForce = spec.minimumBreakingStrength * breakMargin
        };
        j.xDrive = j.yDrive = j.zDrive = drive;

        // Bending: rope resists bending weakly and proportionally to its own
        // stiffness. Free rotation with a soft restoring drive reproduces the
        // difference between stiff steel rope and limp synthetic line.
        j.angularXMotion = j.angularYMotion = j.angularZMotion = ConfigurableJointMotion.Free;

        var bendDrive = new JointDrive
        {
            positionSpring = k * spec.bendStiffness * 0.02f,
            positionDamper = c * spec.bendStiffness,
            maximumForce = Mathf.Infinity
        };
        j.angularXDrive = j.angularYZDrive = bendDrive;

        // Zero target: the joint's rest state is "nodes together", and the limit
        // sphere is what stops them separating. Gravity and tension do the rest.
        j.targetPosition = Vector3.zero;
        j.targetRotation = Quaternion.identity;

        // Projection is the safety net: whatever the iterative solver could not
        // resolve this step gets clamped back inside the stretch limit, so the
        // rope can never quietly grow to twice its length under load.
        // Projection teleports bodies to satisfy a joint, which means it creates
        // velocity out of nothing. On a long chain it fires constantly and turns
        // into an energy pump -- measured here as a rope that accelerated itself
        // to 350 m/s while sitting idle. Only enable it when the chain is
        // actually carrying load and needs the safety net; a visual chain is
        // better off slightly imperfect than exploding.
        if (useJointProjection && loadCarryingEnds)
        {
            j.projectionMode = JointProjectionMode.PositionAndRotation;
            j.projectionDistance = segLen * 0.5f;
            j.projectionAngle = 180f;
        }
        else j.projectionMode = JointProjectionMode.None;

        // Chain joints never break individually. A single joint quietly
        // vanishing mid-chain leaves the rope in two disconnected halves with
        // nothing tracking it, which looks like a physics explosion rather than
        // a rope failure. Rope failure is decided as a whole in CheckFailure().
        j.breakForce = Mathf.Infinity;
        j.breakTorque = Mathf.Infinity;
    }

    /// <summary>
    /// Caps the mass ratio the solver sees across a joint. See
    /// <see cref="maxSolverMassRatio"/> for why this is necessary and why it
    /// does not falsify the physics.
    /// </summary>
    void BalanceSolverMass(ConfigurableJoint j, float ownMass, float otherMass)
    {
        j.massScale = 1f;
        j.connectedMassScale = 1f;

        if (ownMass <= 1e-5f || otherMass <= 1e-5f) return;

        // massScale multiplies a body's INVERSE mass, so a value below 1 makes
        // that body heavier in the solver's eyes.
        //
        // Always stiffen the LIGHT side. Lightening the heavy side would work
        // out to the same ratio but is physically wrong: it would let a 1.4 kg
        // rope fling a 1,200 kg vehicle across the map.
        float ratio = otherMass / ownMass;

        if (ratio > maxSolverMassRatio)
            j.massScale = maxSolverMassRatio / ratio;          // rope node heavier
        else if (ratio < 1f / maxSolverMassRatio)
            j.connectedMassScale = maxSolverMassRatio * ratio; // other side heavier
    }

    void PatchSegmentGeometry()
    {
        float segLen = SegmentLength;

        // Spread the rope's true total mass over the nodes rather than giving
        // every node a full segment's worth, which would overstate the rope's
        // mass by one segment and make it sag too hard.
        float segMass = Mathf.Max(1e-4f,
            Spec.MassForLength(cableLength) / Mathf.Max(1, _nodes.Count));

        for (int i = 0; i < _nodes.Count; i++)
        {
            if (_nodes[i] == null) continue;

            var rb = _rbs[i];
            if (rb != null && !rb.isKinematic) rb.mass = segMass;

            var col = _nodes[i].GetComponent<CapsuleCollider>();
            if (col != null)
            {
                col.radius = Spec.diameter * 0.5f;
                col.height = segLen + Spec.diameter;
                col.center = new Vector3(0f, 0f, segLen * 0.5f);
            }
        }

        foreach (var j in _joints)
            if (j != null) ApplyJointPhysics(j, segLen);

        // Node masses just changed, so the load-end mass ratio has too.
        foreach (var j in _anchorJoints)
        {
            if (j == null || j.connectedBody == null) continue;
            var own = j.GetComponent<Rigidbody>();
            if (own != null) BalanceSolverMass(j, own.mass, j.connectedBody.mass);
        }
    }

    /// <summary>
    /// Attaches a rope end to an anchor with a real joint, and returns it.
    ///
    /// Both ends are jointed rather than pinned, for two reasons:
    ///
    ///  * A kinematic pin lets the rope hold a load but never pull it, which
    ///    defeats the point of a winch. The load has to feel the rope.
    ///  * A kinematic body reports no constraint force, so a pinned drum end
    ///    reads zero tension forever. The winch would then never feel its load:
    ///    the motor would sit at no-load current while dragging a vehicle, and
    ///    the entire torque-balance feedback loop would be dead. Reading force
    ///    out of a real joint is what closes that loop.
    ///
    /// For a static anchor a kinematic proxy body is created to joint against.
    /// </summary>
    ConfigurableJoint AttachEnd(Rigidbody rb, Transform anchor)
    {
        // Visual-only mode: the end simply rides the anchor. No force is
        // exchanged, so no mass ratio can destabilise the solver.
        if (!loadCarryingEnds)
        {
            rb.isKinematic = true;
            var tr = rb.gameObject.GetComponent<CableAnchorTracker>();
            if (tr == null) tr = rb.gameObject.AddComponent<CableAnchorTracker>();
            tr.target = anchor;
            return null;
        }

        var anchorRb = anchor.GetComponentInParent<Rigidbody>();

        if (anchorRb == null || anchorRb.isKinematic)
            anchorRb = GetOrCreateStaticProxy(anchor);

        rb.isKinematic = false;
        rb.position = anchor.position;

        var j = rb.gameObject.AddComponent<ConfigurableJoint>();
        j.connectedBody = anchorRb;
        j.autoConfigureConnectedAnchor = false;
        j.anchor = Vector3.zero;
        j.connectedAnchor = anchorRb.transform.InverseTransformPoint(anchor.position);
        j.enablePreprocessing = false;

        // A hook, shackle or drum termination carries load but swivels freely.
        j.xMotion = j.yMotion = j.zMotion = ConfigurableJointMotion.Locked;
        j.angularXMotion = j.angularYMotion = j.angularZMotion =
            ConfigurableJointMotion.Free;

        if (useJointProjection)
        {
            j.projectionMode = JointProjectionMode.PositionAndRotation;
            j.projectionDistance = 0.05f;
            j.projectionAngle = 180f;
        }

        BalanceSolverMass(j, rb.mass, anchorRb.mass);

        // Must not break on its own: a silently detached end would leave the
        // winch pulling on nothing for the rest of the session. Rope failure is
        // decided as a whole in CheckFailure().
        j.breakForce = Mathf.Infinity;
        j.breakTorque = Mathf.Infinity;

        _anchorJoints.Add(j);
        return j;
    }

    /// <summary>
    /// Builds (or reuses) a kinematic Rigidbody that follows a static anchor,
    /// so the rope has something jointable to pull against.
    /// </summary>
    Rigidbody GetOrCreateStaticProxy(Transform anchor)
    {
        var existing = anchor.GetComponentInChildren<CableAnchorTracker>();
        if (existing != null && existing.transform.parent == anchor)
            return existing.GetComponent<Rigidbody>();

        var go = new GameObject($"RopeAnchorProxy_{anchor.name}");
        go.transform.SetParent(anchor, false);
        go.transform.localPosition = Vector3.zero;
        go.transform.localRotation = Quaternion.identity;

        var rb = go.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;
        rb.mass = 1f;

        var tracker = go.AddComponent<CableAnchorTracker>();
        tracker.target = anchor;

        return rb;
    }

    /// <summary>
    /// Draws the rope at the diameter it actually is. The mesh builder shipped
    /// with a 25 mm radius, which is a 50 mm rope: five times the 3/8" line the
    /// rest of the sim is solving.
    /// </summary>
    void ApplyVisualRadius()
    {
        if (!matchMeshToRopeDiameter) return;
        var builder = GetComponent<CableMeshBuilder>();
        if (builder != null) builder.SetRadius(Spec.diameter * 0.5f);
    }

    void RefreshRendererBuffer()
    {
        if (_rendererBuffer == null || _rendererBuffer.Length != _nodes.Count)
            _rendererBuffer = new Transform[_nodes.Count];
        for (int i = 0; i < _nodes.Count; i++)
            _rendererBuffer[i] = _nodes[i] != null ? _nodes[i].transform : null;
        if (_renderer != null) _renderer.nodes = _rendererBuffer;
    }

    void Snapshot()
    {
        _builtLength = cableLength;
        _builtStartAnchor = startAnchor;
        _builtEndAnchor = endAnchor;
        _builtSpec = rope;
        _builtSegmentDiameters = segmentLengthInDiameters;
        _builtStiffnessScale = stiffnessScale;
        _builtStretchLimit = stretchLimit;
    }

    // ------------------------------------------------------------------
    // Simulation
    // ------------------------------------------------------------------

    void FixedUpdate()
    {
        if (!_built) return;

        if (GuardAgainstBlowUp()) return;

        MeasureTension();
        ApplyExtraGravity();
        if (useAerodynamicDrag) ApplyAerodynamicDrag();
        ClampNodeSpeeds();
        if (tensionShaping && !HasFailed) ApplyTensionShape(Time.fixedDeltaTime);
        if (simulateRopeFailure) CheckFailure();
    }

    /// <summary>
    /// Blends the chain onto the shape its tension implies.
    ///
    /// A line of weight w N/m pulled to tension T across a span L hangs in a
    /// catenary whose mid sag is s = w*L^2/(8*T) for the shallow sags that
    /// matter here. That is the whole model, and it is the standard result used
    /// to size real spans.
    ///
    /// Two sags are computed and the smaller one wins:
    ///
    ///   sTension    what the tension permits, w*L^2/(8*T)
    ///   sGeometric  what the rope length permits. A parabola of sag s over span
    ///               L has arc length L*(1 + 8/3*(s/L)^2), so a rope longer than
    ///               its span has to bow by at least
    ///               s = L*sqrt(3/8*(ropeLength/L - 1)) no matter how hard it is
    ///               pulled.
    ///
    /// When the geometric sag is the smaller one the rope is simply slack and
    /// tension is not shaping anything, so authority drops to zero and the
    /// physics chain is left alone to coil and swing. When tension is the
    /// binding constraint, authority rises and the chain is drawn onto the
    /// curve. That ratio is the blend, so the handover is continuous and needs
    /// no threshold.
    ///
    /// Node positions are written directly rather than forced. The chain is
    /// visual, and the force that would do this honestly is stiff: at 9 kN over
    /// 60 g nodes the straightening spring runs at ~1,000 rad/s, which a 5 ms
    /// step cannot integrate. Setting the shape is stable at any tension.
    /// </summary>
    void ApplyTensionShape(float dt)
    {
        int n = _nodes.Count;
        if (n < 3) { ShapeAuthority = 0f; return; }

        Vector3 a = _nodes[0].transform.position;
        Vector3 b = _nodes[n - 1].transform.position;
        Vector3 chord = b - a;
        float span = chord.magnitude;
        if (span < 1e-4f) { ShapeAuthority = 0f; return; }

        Vector3 g = Physics.gravity * gravityScale;
        float w = Spec.linearDensity * g.magnitude;          // N/m of rope

        // Only the external load path counts here. The chain's own joint forces
        // are not a tension measurement: a rope hanging under its own weight
        // reads a couple of hundred newtons of limit and contact impulse at the
        // end joint, which fed in here would flatten a completely slack rope by
        // a third. Whatever solves the load path is the only thing that knows
        // the real tension, so with nothing reporting, nothing is shaped.
        float tension = _hasExternalTension ? _externalTension : TensionAtStart;

        // Sag the tension allows.
        float sTension = tension > 1e-3f && w > 1e-6f
            ? w * span * span / (8f * tension)
            : Mathf.Infinity;

        // Sag the rope length forces regardless of tension.
        float excess = cableLength / span - 1f;
        float sGeometric = excess > 0f ? span * Mathf.Sqrt(0.375f * excess) : 0f;

        TensionSag = Mathf.Min(sTension, sGeometric);

        // Tension only owns the shape while it is the tighter of the two limits.
        float target = sGeometric > 1e-6f
            ? Mathf.Clamp01(1f - sTension / sGeometric)
            : 1f;

        float k = shapeBlendTime <= 0f ? 1f : 1f - Mathf.Exp(-dt / shapeBlendTime);
        ShapeAuthority = Mathf.Lerp(ShapeAuthority, target, k);

        float blend = ShapeAuthority;
        if (blend < 1e-3f) return;

        // Sag hangs along gravity, but only the part of gravity across the span
        // bows the rope. A vertical rope has no sag at all.
        Vector3 down = Vector3.ProjectOnPlane(g, chord / span);
        if (down.sqrMagnitude < 1e-8f) down = Vector3.zero;
        else down.Normalize();

        float sag = TensionSag;

        for (int i = 1; i < n - 1; i++)
        {
            var rb = _rbs[i];
            if (rb == null || rb.isKinematic) continue;

            float u = i / (float)(n - 1);
            Vector3 want = a + chord * u + down * (4f * sag * u * (1f - u));

            rb.position = Vector3.Lerp(rb.position, want, blend);

            // Whatever velocity the solver gave the node was heading somewhere
            // else. Leaving it in place means the node fights back out of the
            // curve every step and the rope buzzes along its own length.
            rb.linearVelocity *= 1f - blend;
            rb.angularVelocity *= 1f - blend;
        }
    }

    void ClampNodeSpeeds()
    {
        float maxSq = maxNodeSpeed * maxNodeSpeed;
        for (int i = 0; i < _rbs.Count; i++)
        {
            var rb = _rbs[i];
            if (rb == null || rb.isKinematic) continue;

            Vector3 v = rb.linearVelocity;
            if (float.IsNaN(v.x) || float.IsInfinity(v.x))
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                continue;
            }
            if (v.sqrMagnitude > maxSq)
                rb.linearVelocity = v.normalized * maxNodeSpeed;
        }
    }

    /// <summary>
    /// Detects a chain that has come apart -- a broken joint, a NaN, a node
    /// flung to infinity -- and rebuilds it rather than letting a corrupt rope
    /// drag the rest of the scene into nonsense. Should never fire; if it does,
    /// something upstream is wrong and the warning says so.
    /// </summary>
    bool GuardAgainstBlowUp()
    {
        if (HasFailed || _nodes.Count < 2) return false;

        float measured = MeasuredLength;
        bool bad = float.IsNaN(measured) || float.IsInfinity(measured) ||
                   measured > cableLength * 4f + 1f;

        if (!bad) return false;

        Debug.LogWarning($"[Rope] Chain integrity lost (measured {measured:F1} m " +
                         $"vs nominal {cableLength:F2} m). Rebuilding.", this);
        Build();
        return true;
    }

    void Update()
    {
        if (!Application.isPlaying || !_built) return;

        // Structural changes require a rebuild; parameter changes are patched.
        if (startAnchor != _builtStartAnchor ||
            endAnchor != _builtEndAnchor ||
            rope != _builtSpec ||
            !Mathf.Approximately(segmentLengthInDiameters, _builtSegmentDiameters))
        {
            Build();
            return;
        }

        if (!Mathf.Approximately(cableLength, _builtLength))
        {
            SetLength(cableLength);
            return;
        }

        if (!Mathf.Approximately(stiffnessScale, _builtStiffnessScale) ||
            !Mathf.Approximately(stretchLimit, _builtStretchLimit))
        {
            PatchSegmentGeometry();
            _builtStiffnessScale = stiffnessScale;
            _builtStretchLimit = stretchLimit;
        }
    }

    /// <summary>
    /// Reads the constraint force out of each joint. ConfigurableJoint.currentForce
    /// is the force PhysX applied to satisfy the joint last step, which is exactly
    /// the rope tension in that segment.
    /// </summary>
    /// <summary>
    /// True constraint force across a joint (N).
    ///
    /// currentForce is reported in the solver's scaled frame, so a joint that
    /// uses mass scaling reports a force inflated by exactly the scale factor.
    /// On the rope-to-load joint that is a factor of ~200, which would read as
    /// half a million pounds of line pull. Undoing the scale recovers the real
    /// force. Unscaled joints have both factors at 1 and are unaffected.
    /// </summary>
    static float TrueForce(ConfigurableJoint j)
    {
        if (j == null) return 0f;
        return j.currentForce.magnitude * j.massScale * j.connectedMassScale;
    }

    void MeasureTension()
    {
        float peak = 0f;
        for (int i = 0; i < _joints.Count; i++)
        {
            float f = TrueForce(_joints[i]);
            if (f > peak) peak = f;
        }

        // The end joints are the truest readings: one is what the drum pulls
        // against, the other is what the load feels.
        float rawStart = _startJoint != null ? TrueForce(_startJoint)
                       : (_joints.Count > 0 ? TrueForce(_joints[0]) : 0f);

        float rawEnd = _endJoint != null ? TrueForce(_endJoint)
                     : (_joints.Count > 0 ? TrueForce(_joints[_joints.Count - 1]) : 0f);

        if (rawStart > peak) peak = rawStart;
        if (rawEnd > peak) peak = rawEnd;

        // When something else solves the load path, its number is the real
        // tension and the chain's joint forces are only the rope's own weight.
        if (_hasExternalTension)
        {
            rawStart = Mathf.Max(rawStart, _externalTension);
            rawEnd = Mathf.Max(rawEnd, _externalTension);
            peak = Mathf.Max(peak, _externalTension);
        }

        // Iterative solvers produce single-step force spikes that are numerical,
        // not physical. Feeding those straight into the winch drivetrain would
        // make the motor stutter and could trip a rope failure that never
        // happened, so the readings the rest of the sim consumes are smoothed
        // over a few milliseconds.
        float a = tensionSmoothing <= 0f
            ? 1f
            : 1f - Mathf.Exp(-Time.fixedDeltaTime / tensionSmoothing);

        TensionAtStart = Mathf.Lerp(TensionAtStart, rawStart, a);
        TensionAtEnd = Mathf.Lerp(TensionAtEnd, rawEnd, a);
        PeakTension = Mathf.Lerp(PeakTension, peak, a);
    }

    void ApplyExtraGravity()
    {
        if (Mathf.Approximately(gravityScale, 1f)) return;
        Vector3 extra = Physics.gravity * (gravityScale - 1f);
        foreach (var rb in _rbs)
            if (rb != null && !rb.isKinematic)
                rb.AddForce(extra, ForceMode.Acceleration);
    }

    /// <summary>
    /// F = 0.5 * rho * Cd * A * v^2, with A the projected area of the segment
    /// normal to its motion. Gives a swinging rope the right settling time
    /// instead of the uniform exponential decay of Rigidbody.linearDamping.
    /// </summary>
    void ApplyAerodynamicDrag()
    {
        var spec = Spec;
        float area = spec.diameter * SegmentLength;
        float coef = 0.5f * airDensity * spec.airDragCoefficient * area;

        for (int i = 0; i < _rbs.Count; i++)
        {
            var rb = _rbs[i];
            if (rb == null || rb.isKinematic) continue;

            Vector3 v = rb.linearVelocity;
            if (v.sqrMagnitude < 1e-4f) continue;

            // Only the component across the rope axis produces meaningful drag.
            Vector3 axis = rb.transform.forward;
            Vector3 vNormal = v - Vector3.Project(v, axis);

            rb.AddForce(-coef * vNormal.magnitude * vNormal, ForceMode.Force);
        }
    }

    void CheckFailure()
    {
        if (HasFailed) return;

        if (PeakTension < Spec.minimumBreakingStrength * breakMargin)
        {
            _overloadTimer = 0f;
            return;
        }

        _overloadTimer += Time.fixedDeltaTime;
        if (_overloadTimer < overloadDuration) return;

        HasFailed = true;
        Debug.LogWarning($"[Rope] Parted at {PeakTension:F0} N " +
                         $"({PeakTension / Spec.minimumBreakingStrength:P0} of MBS).", this);

        // Part the rope at the most loaded end and let it whip free, which is
        // what actually happens and is the most dangerous event in winching.
        foreach (var j in _anchorJoints)
            if (j != null) Destroy(j);
        _anchorJoints.Clear();
        _startJoint = null;
        _endJoint = null;

        foreach (var rb in _rbs)
            if (rb != null) rb.isKinematic = false;
    }

    void OnDrawGizmosSelected()
    {
        if (_nodes.Count == 0) return;

        for (int i = 0; i < _nodes.Count; i++)
        {
            if (_nodes[i] == null) continue;
            bool anchored = (i == 0 && startAnchor != null) ||
                            (i == _nodes.Count - 1 && endAnchor != null);
            Gizmos.color = anchored
                ? new Color(0.2f, 0.8f, 1f, 0.7f)
                : Color.Lerp(new Color(0.3f, 1f, 0.3f, 0.35f),
                             new Color(1f, 0.2f, 0.1f, 0.9f),
                             LoadFraction);
            Gizmos.DrawWireSphere(_nodes[i].transform.position, Spec.diameter);
        }
    }
}

/// <summary>
/// Keeps an anchored rope node glued to a target Transform every physics step.
/// Created automatically by CableSimulator.
/// </summary>
public class CableAnchorTracker : MonoBehaviour
{
    public Transform target;
    private Rigidbody _rb;

    void Awake() => _rb = GetComponent<Rigidbody>();

    void FixedUpdate()
    {
        if (target == null || _rb == null) return;
        _rb.MovePosition(target.position);
        _rb.MoveRotation(target.rotation);
    }
}
