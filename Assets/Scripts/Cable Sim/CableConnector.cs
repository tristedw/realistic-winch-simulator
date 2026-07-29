using UnityEngine;
using UnityEngine.Events;

public class CableConnector : MonoBehaviour
{
    [Header("Socket")]
    public float snapRadius = 0.12f;
    public float breakForce = 800f;

    [Header("Events")]
    public UnityEvent OnConnected;
    public UnityEvent OnDisconnected;

    public Rigidbody ConnectedNode { get; private set; }

    FixedJoint joint;
    Rigidbody socketBody;

    void Awake()
    {
        socketBody = GetComponent<Rigidbody>();
        if (socketBody == null) socketBody = GetComponentInParent<Rigidbody>();
    }

    public bool TrySnap(Rigidbody nodeRb)
    {
        if (nodeRb == null) return false;
        if (ConnectedNode != null) return false;
        if (!CanSnapTo(nodeRb)) return false;

        float dist = Vector3.Distance(nodeRb.position, transform.position);
        if (dist > snapRadius) return false;

        Connect(nodeRb);
        return true;
    }

    /// <summary>
    /// A node is only snappable if it is free. Two cases have to be rejected or
    /// the joint fights something that cannot lose:
    ///
    ///  * The node is already anchored (kinematic, driven by CableAnchorTracker).
    ///    Connect() teleports the node to the socket, the tracker teleports it
    ///    straight back next step, and the FixedJoint then has a permanent
    ///    position violation against a kinematic body -- which means unbounded
    ///    force on whatever the socket is attached to.
    ///
    ///  * The cable end is anchored to this socket's own body. That is a joint
    ///    from a rigidbody back to itself through the rope, so the socket drags
    ///    itself. This is what happened on the winch load: the load cube's own
    ///    socket grabbed the rope end that was already tied to the load, launched
    ///    it, broke the joint at breakForce, re-snapped the next FixedUpdate and
    ///    repeated forever.
    /// </summary>
    bool CanSnapTo(Rigidbody nodeRb)
    {
        if (nodeRb.isKinematic) return false;
        if (nodeRb.GetComponent<CableAnchorTracker>() != null) return false;
        if (socketBody != null && nodeRb == socketBody) return false;

        var sim = nodeRb.GetComponentInParent<CableSimulator>();
        if (sim != null && (SharesBodyWith(sim.startAnchor) || SharesBodyWith(sim.endAnchor)))
            return false;

        return true;
    }

    /// <summary>True if an anchor transform hangs off the same body as this socket.</summary>
    bool SharesBodyWith(Transform anchor)
    {
        if (anchor == null) return false;
        if (anchor == transform || anchor.IsChildOf(transform)) return true;

        if (socketBody == null) return false;
        var theirs = anchor.GetComponentInParent<Rigidbody>();
        return theirs != null && theirs == socketBody;
    }

    void Connect(Rigidbody nodeRb)
    {
        ConnectedNode = nodeRb;

        // snap cable node to socket
        nodeRb.position = transform.position;
        nodeRb.linearVelocity = Vector3.zero;
        nodeRb.angularVelocity = Vector3.zero;

        joint = gameObject.AddComponent<FixedJoint>();

        joint.connectedBody = nodeRb;
        joint.breakForce = breakForce;
        joint.breakTorque = Mathf.Infinity;

        joint.autoConfigureConnectedAnchor = true;

        OnConnected?.Invoke();
    }

    public void Disconnect()
    {
        if (joint != null)
            Destroy(joint);

        ConnectedNode = null;
        joint = null;

        OnDisconnected?.Invoke();
    }

    void OnJointBreak(float force)
    {
        Debug.Log($"Cable broke at {force}");

        joint = null;
        ConnectedNode = null;

        OnDisconnected?.Invoke();
    }

    void OnDrawGizmos()
    {
        bool connected = ConnectedNode != null;

        Gizmos.color = connected
            ? new Color(0.2f, 1f, 0.3f, 0.35f)
            : new Color(1f, 1f, 0.2f, 0.2f);

        Gizmos.DrawWireSphere(transform.position, snapRadius);

        Gizmos.color = connected
            ? new Color(0.2f, 1f, 0.3f, 0.8f)
            : new Color(1f, 1f, 0.2f, 0.6f);

        Gizmos.DrawSphere(transform.position, snapRadius * 0.12f);

        if (connected)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, ConnectedNode.position);
        }
    }
}