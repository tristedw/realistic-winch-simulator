using UnityEngine;

[RequireComponent(typeof(CableMeshBuilder))]
public class CableRenderer : MonoBehaviour
{
    [HideInInspector] public Transform[] nodes;

    private CableMeshBuilder _builder;
    private Vector3[] _positions;

    void Awake()
    {
        _builder = GetComponent<CableMeshBuilder>();
    }

    void LateUpdate()
    {
        if (nodes == null || nodes.Length < 2) return;

        // Resize buffer only when node count changes
        if (_positions == null || _positions.Length != nodes.Length)
            _positions = new Vector3[nodes.Length];

        for (int i = 0; i < nodes.Length; i++)
            _positions[i] = nodes[i].position;

        _builder.Rebuild(_positions);
    }
}
