using UnityEngine;

[RequireComponent(typeof(CableRenderer))]
public class CableSimulator : MonoBehaviour
{
    [Header("Cable Shape")]
    public int segments = 12;

    [Tooltip("Total cable length in world units.")]
    public float cableLength = 2f;

    [Header("Node Physics")]
    [Tooltip("Mass of each Rigidbody node. Lower = floatier, higher = heavier sag.")]
    public float nodeMass = 0.05f;

    [Tooltip("Linear drag on each node. Higher values slow down swinging.")]
    [Range(0f, 10f)] public float nodeDrag = 0.5f;

    [Tooltip("Angular drag on each node. Higher values damp twisting.")]
    [Range(0f, 20f)] public float nodeAngularDrag = 5f;

    [Range(-2f, 2f)] public float gravityScale = 1f;

    [Header("Joint Stiffness")]
    [Tooltip("Spring force resisting positional deviation from rest length. ")]
    [Range(0f, 5000f)] public float positionSpring = 800f;

    [Tooltip("Damping on the linear spring. Higher reduces oscillation.")]
    [Range(0f, 200f)] public float positionDamper = 40f;

    [Tooltip("Spring force resisting angular deviation. Higher = stiffer bends.")]
    [Range(0f, 500f)] public float angularSpring = 100f;

    [Tooltip("Damping on the angular spring. Higher reduces wobble.")]
    [Range(0f, 100f)] public float angularDamper = 10f;

    [Header("Stretch & Stiffness")]
    [Tooltip("Hard maximum stretch per segment as a fraction of segment length. " +
             "0 = no stretch allowed (rigid), 0.1 = 10% stretch. " +
             "This is the PRIMARY fix for anchor droop (0.02–0.05).")]
    [Range(0f, 0.5f)] public float stretchLimit = 0.02f;

    [Tooltip("Contact distance for the linear limit spring. Lower = harder stop.")]
    [Range(0f, 0.1f)] public float limitContactDistance = 0.01f;

    [Tooltip("Spring stiffness at the hard stretch limit. Higher = snappier recovery.")]
    [Range(0f, 10000f)] public float limitSpring = 5000f;

    [Tooltip("Damping at the hard stretch limit.")]
    [Range(0f, 500f)] public float limitDamper = 50f;

    [Header("Break Forces")]
    public float breakForce = Mathf.Infinity;
    public float breakTorque = Mathf.Infinity;

    [Header("Collision")]
    [Tooltip("Radius of the CapsuleCollider on each node.")]
    public float colliderRadius = 0.03f;

    public LayerMask collisionLayers = ~0;

    [Header("Anchors")]
    [Tooltip("If set, the first node is pinned to this Transform.")]
    public Transform startAnchor;

    [Tooltip("If set, the last node is pinned to this Transform.")]
    public Transform endAnchor;

    public Rigidbody GetNode(int i) => (_rbs != null && i >= 0 && i < _rbs.Length) ? _rbs[i] : null;
    public int NodeCount => _nodes != null ? _nodes.Length : 0;

    private GameObject[] _nodes;
    private Rigidbody[] _rbs;
    private CableRenderer _renderer;

    // Snapshots for change detection
    private int _builtSegments;
    private float _builtLength;
    private Transform _builtStartAnchor;
    private Transform _builtEndAnchor;

    private float _builtPositionSpring;
    private float _builtPositionDamper;
    private float _builtAngularSpring;
    private float _builtAngularDamper;
    private float _builtStretchLimit;
    private float _builtLimitContactDistance;
    private float _builtLimitSpring;
    private float _builtLimitDamper;
    private float _builtBreakForce;
    private float _builtBreakTorque;
    private float _builtNodeMass;
    private float _builtNodeDrag;
    private float _builtNodeAngularDrag;
    private float _builtColliderRadius;
    private float _builtGravityScale;

    void Awake()
    {
        _renderer = GetComponent<CableRenderer>();
    }

    void Start()
    {
        Build();
    }

    void Update()
    {
        if (!Application.isPlaying) return;
        CheckForInspectorChanges();

        if (_rbs != null && gravityScale != 1f)
        {
            Vector3 extraGravity = Physics.gravity * (gravityScale - 1f);
            foreach (var rb in _rbs)
                if (rb != null && !rb.isKinematic)
                    rb.AddForce(extraGravity, ForceMode.Acceleration);
        }
    }

    void OnDestroy() => Teardown();

    void CheckForInspectorChanges()
    {
        bool needsRebuild =
            segments != _builtSegments ||
            cableLength != _builtLength ||
            startAnchor != _builtStartAnchor ||
            endAnchor != _builtEndAnchor;

        if (needsRebuild)
        {
            Teardown();
            Build();
            return;
        }

        bool needsJointPatch =
            positionSpring != _builtPositionSpring ||
            positionDamper != _builtPositionDamper ||
            angularSpring != _builtAngularSpring ||
            angularDamper != _builtAngularDamper ||
            stretchLimit != _builtStretchLimit ||
            limitContactDistance != _builtLimitContactDistance ||
            limitSpring != _builtLimitSpring ||
            limitDamper != _builtLimitDamper ||
            breakForce != _builtBreakForce ||
            breakTorque != _builtBreakTorque;

        if (needsJointPatch) PatchJoints();
        if (nodeMass != _builtNodeMass) PatchMass();
        if (nodeDrag != _builtNodeDrag ||
            nodeAngularDrag != _builtNodeAngularDrag) PatchDrag();
        if (colliderRadius != _builtColliderRadius) PatchColliders();
        if (gravityScale != _builtGravityScale) _builtGravityScale = gravityScale;
    }

    void Build()
    {
        int count = segments + 1;
        float segLen = cableLength / Mathf.Max(1, segments);

        _nodes = new GameObject[count];
        _rbs = new Rigidbody[count];

        Vector3 startPos = startAnchor != null ? startAnchor.position : transform.position;
        Vector3 dir = endAnchor != null
            ? (endAnchor.position - startPos).normalized
            : -transform.up;

        var transforms = new Transform[count];

        for (int i = 0; i < count; i++)
        {
            var go = new GameObject($"CableNode_{i}");
            go.transform.SetParent(transform);
            go.transform.position = startPos + dir * segLen * i;
            go.layer = gameObject.layer;

            var rb = go.AddComponent<Rigidbody>();
            rb.mass = nodeMass;
            rb.linearDamping = nodeDrag;
            rb.angularDamping = nodeAngularDrag;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.useGravity = true;

            if (i < count - 1)
            {
                var col = go.AddComponent<CapsuleCollider>();
                col.radius = colliderRadius;
                col.height = segLen + colliderRadius * 2f;
                col.direction = 2;
                col.center = new Vector3(0f, 0f, segLen * 0.5f);
            }

            _nodes[i] = go;
            _rbs[i] = rb;
            transforms[i] = go.transform;
        }

        // Connect joints node 0 is the anchor end, chain flows outward
        for (int i = 1; i < count; i++)
            ConnectJoint(_rbs[i], _rbs[i - 1], segLen);

        // Pin anchors AFTER joints so FixedJoint overrides the spring on node 0/last
        if (startAnchor != null) PinToAnchor(_rbs[0], startAnchor);
        if (endAnchor != null) PinToAnchor(_rbs[count - 1], endAnchor);

        _renderer.nodes = transforms;
        SnapshotBuiltValues();
    }

    void Teardown()
    {
        if (_nodes == null) return;
        foreach (var n in _nodes)
            if (n != null) Destroy(n);
        _nodes = null;
        _rbs = null;
    }

    void PatchJoints()
    {
        if (_nodes == null) return;

        float segLen = cableLength / Mathf.Max(1, segments);

        var linearDrive = new JointDrive
        {
            positionSpring = positionSpring,
            positionDamper = positionDamper,
            maximumForce = Mathf.Infinity
        };
        var angularDrive = new JointDrive
        {
            positionSpring = angularSpring,
            positionDamper = angularDamper,
            maximumForce = Mathf.Infinity
        };
        var limitSpringVal = new SoftJointLimitSpring
        {
            spring = limitSpring,
            damper = limitDamper
        };
        var linearLimit = new SoftJointLimit
        {
            limit = segLen * stretchLimit,
            contactDistance = limitContactDistance
        };

        for (int i = 1; i < _nodes.Length; i++)
        {
            var joint = _nodes[i].GetComponent<ConfigurableJoint>();
            if (joint == null) continue;

            joint.xDrive = joint.yDrive = joint.zDrive = linearDrive;
            joint.angularXDrive = joint.angularYZDrive = angularDrive;
            joint.linearLimitSpring = limitSpringVal;
            joint.linearLimit = linearLimit;
            joint.breakForce = breakForce;
            joint.breakTorque = breakTorque;
        }

        _builtPositionSpring = positionSpring;
        _builtPositionDamper = positionDamper;
        _builtAngularSpring = angularSpring;
        _builtAngularDamper = angularDamper;
        _builtStretchLimit = stretchLimit;
        _builtLimitContactDistance = limitContactDistance;
        _builtLimitSpring = limitSpring;
        _builtLimitDamper = limitDamper;
        _builtBreakForce = breakForce;
        _builtBreakTorque = breakTorque;
    }

    void PatchMass()
    {
        if (_rbs == null) return;
        foreach (var rb in _rbs)
            if (rb != null) rb.mass = nodeMass;
        _builtNodeMass = nodeMass;
    }

    void PatchDrag()
    {
        if (_rbs == null) return;
        foreach (var rb in _rbs)
        {
            if (rb == null) continue;
            rb.linearDamping = nodeDrag;
            rb.angularDamping = nodeAngularDrag;
        }
        _builtNodeDrag = nodeDrag;
        _builtNodeAngularDrag = nodeAngularDrag;
    }

    void PatchColliders()
    {
        if (_nodes == null) return;
        float segLen = cableLength / Mathf.Max(1, segments);
        for (int i = 0; i < _nodes.Length - 1; i++)
        {
            var col = _nodes[i].GetComponent<CapsuleCollider>();
            if (col == null) continue;
            col.radius = colliderRadius;
            col.height = segLen + colliderRadius * 2f;
        }
        _builtColliderRadius = colliderRadius;
    }

    void SnapshotBuiltValues()
    {
        _builtSegments = segments;
        _builtLength = cableLength;
        _builtStartAnchor = startAnchor;
        _builtEndAnchor = endAnchor;
        _builtPositionSpring = positionSpring;
        _builtPositionDamper = positionDamper;
        _builtAngularSpring = angularSpring;
        _builtAngularDamper = angularDamper;
        _builtStretchLimit = stretchLimit;
        _builtLimitContactDistance = limitContactDistance;
        _builtLimitSpring = limitSpring;
        _builtLimitDamper = limitDamper;
        _builtBreakForce = breakForce;
        _builtBreakTorque = breakTorque;
        _builtNodeMass = nodeMass;
        _builtNodeDrag = nodeDrag;
        _builtNodeAngularDrag = nodeAngularDrag;
        _builtColliderRadius = colliderRadius;
        _builtGravityScale = gravityScale;
    }

    void ConnectJoint(Rigidbody child, Rigidbody parent, float segLen)
    {
        var joint = child.gameObject.AddComponent<ConfigurableJoint>();
        joint.connectedBody = parent;

        // Use Limited motion instead of Free so the chain has a hard length cap.
        // The spring/damper handle oscillation damping; the limit handles load-bearing.
        joint.xMotion = ConfigurableJointMotion.Limited;
        joint.yMotion = ConfigurableJointMotion.Limited;
        joint.zMotion = ConfigurableJointMotion.Limited;

        joint.linearLimitSpring = new SoftJointLimitSpring
        {
            spring = limitSpring,
            damper = limitDamper
        };
        joint.linearLimit = new SoftJointLimit
        {
            limit = segLen * stretchLimit,
            contactDistance = limitContactDistance
        };

        var linearDrive = new JointDrive
        {
            positionSpring = positionSpring,
            positionDamper = positionDamper,
            maximumForce = Mathf.Infinity
        };
        joint.xDrive = joint.yDrive = joint.zDrive = linearDrive;

        joint.angularXMotion = ConfigurableJointMotion.Free;
        joint.angularYMotion = ConfigurableJointMotion.Free;
        joint.angularZMotion = ConfigurableJointMotion.Free;

        var angDrive = new JointDrive
        {
            positionSpring = angularSpring,
            positionDamper = angularDamper,
            maximumForce = Mathf.Infinity
        };
        joint.angularXDrive = joint.angularYZDrive = angDrive;

        joint.targetPosition = new Vector3(0f, 0f, -segLen);
        joint.autoConfigureConnectedAnchor = false;
        joint.connectedAnchor = Vector3.zero;
        joint.anchor = Vector3.zero;

        joint.breakForce = breakForce;
        joint.breakTorque = breakTorque;
    }

    void PinToAnchor(Rigidbody rb, Transform anchor)
    {
        // A kinematic node ignores all forces and holds position absolutely,
        // which prevents the first node from sagging even under heavy chain load.
        rb.isKinematic = true;

        // Move the kinematic node to the exact anchor position each frame
        // via a small helper component, so it follows moving anchors too.
        var tracker = rb.gameObject.AddComponent<CableAnchorTracker>();
        tracker.target = anchor;
    }


    void OnDrawGizmosSelected()
    {
        if (_nodes == null) return;
        float segLen = cableLength / Mathf.Max(1, segments);

        for (int i = 0; i < _nodes.Length; i++)
        {
            if (_nodes[i] == null) continue;
            bool isAnchor = (i == 0 && startAnchor != null) ||
                            (i == _nodes.Length - 1 && endAnchor != null);
            Gizmos.color = isAnchor
                ? new Color(0.2f, 0.8f, 1f, 0.6f)
                : new Color(1f, 0.6f, 0.1f, 0.2f);
            Gizmos.DrawWireSphere(_nodes[i].transform.position, colliderRadius + 0.01f);
        }
    }
}

/// <summary>
/// Lightweight component that keeps an anchored cable node glued
/// to a target Transform every FixedUpdate.
/// Created automatically by CableSimulator.PinToAnchor().
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