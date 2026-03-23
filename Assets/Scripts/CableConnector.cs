using UnityEngine;
using UnityEngine.Events;

public class CableConnector : MonoBehaviour
{
    [Header("Socket")]
    public float snapRadius = 0.12f;
    public float detachForce = 8f;

    [Header("Events")]
    public UnityEvent OnConnected;
    public UnityEvent OnDisconnected;

    [Header("Debug")]
    public bool logSnapAttempts = true;
    public bool logEveryCheck = false;

    public Rigidbody ConnectedNode { get; private set; }

    private FixedJoint _joint;

    public bool TrySnap(Rigidbody nodeRb)
    {
        if (nodeRb == null)
        {
            if (logSnapAttempts)
                Debug.LogWarning($"[Socket:{name}] TrySnap called with null Rigidbody.", this);
            return false;
        }

        if (ConnectedNode != null)
        {
            if (logEveryCheck)
                Debug.Log($"[Socket:{name}] TrySnap already occupied by '{ConnectedNode.gameObject.name}', " +
                          $"ignoring node '{nodeRb.gameObject.name}'.");
            return false;
        }

        float dist = Vector3.Distance(nodeRb.position, transform.position);

        if (logSnapAttempts)
            Debug.Log($"[Socket:{name}] TrySnap node='{nodeRb.gameObject.name}' | " +
                      $"dist={dist:F3} | snapRadius={snapRadius} | " +
                      $"{(dist <= snapRadius ? "IN RANGE connecting" : "out of range")}");

        if (dist > snapRadius) return false;

        Connect(nodeRb);
        return true;
    }

    public void Disconnect()
    {
        if (ConnectedNode == null)
        {
            return;
        }

        string nodeName = ConnectedNode.gameObject.name;

        if (_joint != null)
        {
            Destroy(_joint);
            _joint = null;
        }

        ConnectedNode = null;
        OnDisconnected?.Invoke();
    }

    void Connect(Rigidbody nodeRb)
    {
        // Snap node to exact socket position before locking
        Vector3 prevPos = nodeRb.position;
        nodeRb.MovePosition(transform.position);
        nodeRb.linearVelocity = Vector3.zero;
        nodeRb.angularVelocity = Vector3.zero;

        // Create FixedJoint on the node, connected to a kinematic anchor at the socket
        _joint = nodeRb.gameObject.AddComponent<FixedJoint>();

        float scaledBreakForce = detachForce * nodeRb.mass * 100f;
        _joint.breakForce = scaledBreakForce;
        _joint.breakTorque = Mathf.Infinity;

        var anchor = GetOrCreateAnchor();
        _joint.connectedBody = anchor;

        ConnectedNode = nodeRb;
        OnConnected?.Invoke();
    }

    void Update()
    {
        // Detect joint broken by physics (joint destroyed externally by Unity)
        if (ConnectedNode != null && _joint == null)
        {
            ConnectedNode = null;
            OnDisconnected?.Invoke();
        }
    }

    Rigidbody GetOrCreateAnchor()
    {
        var child = transform.Find("__SocketAnchor");
        if (child != null)
        {
            return child.GetComponent<Rigidbody>();
        }

        var go = new GameObject("__SocketAnchor");
        go.transform.SetParent(transform);
        go.transform.localPosition = Vector3.zero;
        var rb = go.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        return rb;
    }

    void OnDrawGizmos()
    {
        bool connected = ConnectedNode != null;

        // Outer snap radius ring
        Gizmos.color = connected
            ? new Color(0.2f, 1f, 0.3f, 0.35f)
            : new Color(1f, 1f, 0.2f, 0.2f);
        Gizmos.DrawWireSphere(transform.position, snapRadius);

        // Solid centre dot
        Gizmos.color = connected
            ? new Color(0.2f, 1f, 0.3f, 0.8f)
            : new Color(1f, 1f, 0.2f, 0.6f);
        Gizmos.DrawSphere(transform.position, snapRadius * 0.12f);

        // Line to connected node
        if (connected)
        {
            Gizmos.color = new Color(0.2f, 1f, 0.3f, 0.5f);
            Gizmos.DrawLine(transform.position, ConnectedNode.position);
        }

#if UNITY_EDITOR
        // Status label
        string label = connected
            ? $"[CONNECTED]\n{ConnectedNode.gameObject.name}"
            : $"[{name}]\nfree  r={snapRadius:F2}";
        UnityEditor.Handles.Label(transform.position + Vector3.up * (snapRadius + 0.05f), label);
#endif
    }
}