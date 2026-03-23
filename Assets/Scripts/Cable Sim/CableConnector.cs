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
    }

    public bool TrySnap(Rigidbody nodeRb)
    {
        if (nodeRb == null) return false;
        if (ConnectedNode != null) return false;

        float dist = Vector3.Distance(nodeRb.position, transform.position);
        if (dist > snapRadius) return false;

        Connect(nodeRb);
        return true;
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