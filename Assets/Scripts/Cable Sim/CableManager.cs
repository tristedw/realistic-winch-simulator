using System.Collections.Generic;
using UnityEngine;

public class CableManager : MonoBehaviour
{
    public static CableManager Instance { get; private set; }

    [Header("Cable Prefab")]
    [Tooltip("A prefab with CableSimulator + CableRenderer + CableMeshBuilder + CableGrabHandler")]
    public GameObject cablePrefab;

    [Header("Connectors in scene")]
    public List<CableConnector> sockets = new List<CableConnector>();

    private readonly List<CableSimulator> _cables = new List<CableSimulator>();

    [SerializeField] private Transform startAnchor;
    [SerializeField] private Transform endAnchor;
    [SerializeField] private bool spawnCableOnStart = true;

    void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;

        if (spawnCableOnStart)
            SpawnCable(startAnchor != null ? startAnchor.position : transform.position, endAnchor != null ? endAnchor.position : transform.position,
                startAnchor != null ? startAnchor : null, endAnchor != null ? endAnchor : null);
    }

    void FixedUpdate()
    {
        // Check if any free cable endpoint is near a socket
        foreach (var cable in _cables)
        {
            if (cable == null) continue;

            // Only ends that are actually free are offered to sockets. An end
            // held by startAnchor/endAnchor is already attached to something, and
            // snapping it again means a joint fighting the anchor tracker.
            Rigidbody startNode = cable.startAnchor == null ? cable.GetNode(0) : null;
            Rigidbody endNode   = cable.endAnchor == null
                                ? cable.GetNode(cable.NodeCount - 1) : null;

            if (startNode == null && endNode == null) continue;

            foreach (var socket in sockets)
            {
                if (socket == null || socket.ConnectedNode != null) continue;
                if (startNode != null) socket.TrySnap(startNode);
                if (endNode != null) socket.TrySnap(endNode);
            }
        }
    }

    public CableSimulator SpawnCable(Vector3 from, Vector3 to,
                                     Transform startAnchor = null,
                                     Transform endAnchor   = null)
    {
        if (cablePrefab == null)
        {
            Debug.LogError("[CableManager] cablePrefab is not assigned.");
            return null;
        }

        var go  = Instantiate(cablePrefab, from, Quaternion.identity);
        var sim = go.GetComponent<CableSimulator>();

        sim.startAnchor = startAnchor;
        sim.endAnchor   = endAnchor;

        _cables.Add(sim);
        return sim;
    }

    public void DestroyCable(CableSimulator cable)
    {
        if (cable == null) return;
        _cables.Remove(cable);
        Destroy(cable.gameObject);
    }

    public void Register(CableSimulator cable)
    {
        if (!_cables.Contains(cable)) _cables.Add(cable);
    }

    public IReadOnlyList<CableSimulator> Cables => _cables;
}
