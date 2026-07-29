using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class CableMeshBuilder : MonoBehaviour
{
    [Header("Tube Shape")]
    [Tooltip("Smoothness around the tube.")]
    public int radialSegments = 8;

    [Tooltip("Rope radius in metres. Driven from WireRopeSpec.diameter at " +
             "runtime when CableSimulator.matchMeshToRopeDiameter is on, so a " +
             "3/8\" rope is drawn 9.5 mm thick instead of whatever was typed here.")]
    public float radius = 0.00477f;

    private Mesh _mesh;
    private MeshFilter _filter;

    void Awake()
    {
        _filter = GetComponent<MeshFilter>();
        _mesh = new Mesh { name = "CableMesh" };
        _mesh.MarkDynamic();
        _filter.mesh = _mesh;
    }

    public void SetRadius(float r) => radius = Mathf.Max(1e-4f, r);

    public void Rebuild(Vector3[] positions)
    {
        if (positions == null || positions.Length < 2) return;
        if (_mesh == null) return;

        RopeTubeMesh.Build(_mesh, positions, positions.Length,
                           radius, radialSegments, transform);
    }
}
