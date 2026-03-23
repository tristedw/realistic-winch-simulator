using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class CableMeshBuilder : MonoBehaviour
{
    [Header("Tube Shape")]
    public int radialSegments = 8;       // smoothness around the tube
    public float radius = 0.025f;        // cable thickness in world units

    private Mesh _mesh;
    private MeshFilter _filter;

    void Awake()
    {
        _filter = GetComponent<MeshFilter>();
        _mesh   = new Mesh { name = "CableMesh" };
        _filter.mesh = _mesh;
    }

    public void Rebuild(Vector3[] positions)
    {
        if (positions == null || positions.Length < 2) return;

        int segments   = positions.Length - 1;
        int rings      = positions.Length;
        int vertsPerRing = radialSegments + 1;  // +1 to close the seam with matching UV

        var verts  = new Vector3[rings * vertsPerRing];
        var uvs    = new Vector2[rings * vertsPerRing];
        var tris   = new int[segments * radialSegments * 6];

        float totalLength = 0f;
        var cumDist = new float[rings];
        for (int i = 1; i < rings; i++)
        {
            totalLength += Vector3.Distance(positions[i - 1], positions[i]);
            cumDist[i]  = totalLength;
        }

        // Build rings of vertices around each position
        for (int r = 0; r < rings; r++)
        {
            // Compute a stable frame (forward + up) for each ring
            Vector3 forward = GetForward(positions, r);
            Vector3 up      = GetUp(forward);
            Quaternion rot  = Quaternion.LookRotation(forward, up);

            float v = totalLength > 0f ? cumDist[r] / totalLength : 0f;

            for (int s = 0; s <= radialSegments; s++)
            {
                float angle = (s / (float)radialSegments) * Mathf.PI * 2f;
                Vector3 offset = rot * new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f) * radius;

                int idx    = r * vertsPerRing + s;
                verts[idx] = transform.InverseTransformPoint(positions[r] + offset);
                uvs[idx]   = new Vector2(s / (float)radialSegments, v);
            }
        }

        // Stitch triangles
        int t = 0;
        for (int r = 0; r < segments; r++)
        {
            for (int s = 0; s < radialSegments; s++)
            {
                int a = r       * vertsPerRing + s;
                int b = (r + 1) * vertsPerRing + s;
                int c = a + 1;
                int d = b + 1;

                tris[t++] = a; tris[t++] = b; tris[t++] = c;
                tris[t++] = c; tris[t++] = b; tris[t++] = d;
            }
        }

        _mesh.Clear();
        _mesh.vertices  = verts;
        _mesh.uv        = uvs;
        _mesh.triangles = tris;
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    Vector3 GetForward(Vector3[] pos, int i)
    {
        if (i == 0)               return (pos[1] - pos[0]).normalized;
        if (i == pos.Length - 1)  return (pos[i] - pos[i - 1]).normalized;
        return ((pos[i + 1] - pos[i - 1]) * 0.5f).normalized;
    }

    Vector3 GetUp(Vector3 forward)
    {
        // Avoid gimbal lock when cable points straight up
        Vector3 reference = Mathf.Abs(Vector3.Dot(forward, Vector3.up)) > 0.99f
            ? Vector3.right
            : Vector3.up;
        return Vector3.Cross(Vector3.Cross(forward, reference), forward).normalized;
    }
}
