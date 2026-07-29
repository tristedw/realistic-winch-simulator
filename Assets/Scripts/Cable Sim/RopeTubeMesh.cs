using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Builds a swept tube around a polyline. Shared by the free rope
/// (<see cref="CableMeshBuilder"/>) and the drum wrap
/// (<see cref="WinchDrumRopeVisual"/>) so both render the same rope.
///
/// Frames are parallel transported along the curve instead of being rebuilt
/// from a fixed world up vector at every ring. Rebuilding them independently
/// makes the tube twist about its own axis wherever the curve turns, which on a
/// helix wound round a drum shows up as the rope visibly corkscrewing, and on a
/// swinging rope shows up as the texture spinning.
/// </summary>
public static class RopeTubeMesh
{
    /// <summary>
    /// Writes a tube of <paramref name="radius"/> around the first
    /// <paramref name="count"/> entries of <paramref name="points"/>.
    /// Vertices are emitted in <paramref name="space"/>'s local coordinates,
    /// or world space if it is null.
    /// </summary>
    public static void Build(Mesh mesh, IList<Vector3> points, int count,
                             float radius, int radialSegments, Transform space)
    {
        if (mesh == null || points == null) return;

        radialSegments = Mathf.Max(3, radialSegments);
        count = Mathf.Min(count, points.Count);

        if (count < 2) { mesh.Clear(); return; }

        int vertsPerRing = radialSegments + 1;   // duplicate seam vertex for UVs
        int segments = count - 1;

        var verts = new Vector3[count * vertsPerRing];
        var uvs = new Vector2[count * vertsPerRing];
        var tris = new int[segments * radialSegments * 6];

        // Arc length for the V coordinate, so a repeating rope texture keeps a
        // constant lay regardless of how the polyline is spaced.
        var cum = new float[count];
        float total = 0f;
        for (int i = 1; i < count; i++)
        {
            total += Vector3.Distance(points[i - 1], points[i]);
            cum[i] = total;
        }

        Vector3 tangent = SafeDir(points[1] - points[0], Vector3.forward);
        Vector3 normal = Perpendicular(tangent);

        for (int r = 0; r < count; r++)
        {
            Vector3 next = r == 0 ? tangent
                         : r == count - 1 ? SafeDir(points[r] - points[r - 1], tangent)
                         : SafeDir(points[r + 1] - points[r - 1], tangent);

            // Rotate the frame by the same rotation that took the old tangent to
            // the new one. That is the parallel transport step.
            normal = Quaternion.FromToRotation(tangent, next) * normal;
            tangent = next;

            normal = Vector3.ProjectOnPlane(normal, tangent);
            if (normal.sqrMagnitude < 1e-8f) normal = Perpendicular(tangent);
            normal.Normalize();

            Vector3 binormal = Vector3.Cross(tangent, normal);
            float v = total > 1e-6f ? cum[r] / total : 0f;

            for (int s = 0; s <= radialSegments; s++)
            {
                float a = s / (float)radialSegments * Mathf.PI * 2f;
                Vector3 p = points[r] +
                            (normal * Mathf.Cos(a) + binormal * Mathf.Sin(a)) * radius;

                int idx = r * vertsPerRing + s;
                verts[idx] = space != null ? space.InverseTransformPoint(p) : p;
                uvs[idx] = new Vector2(s / (float)radialSegments, v);
            }
        }

        int t = 0;
        for (int r = 0; r < segments; r++)
        {
            for (int s = 0; s < radialSegments; s++)
            {
                int a = r * vertsPerRing + s;
                int b = (r + 1) * vertsPerRing + s;
                tris[t++] = a; tris[t++] = b; tris[t++] = a + 1;
                tris[t++] = a + 1; tris[t++] = b; tris[t++] = b + 1;
            }
        }

        mesh.Clear();
        mesh.indexFormat = verts.Length > 65000
            ? UnityEngine.Rendering.IndexFormat.UInt32
            : UnityEngine.Rendering.IndexFormat.UInt16;
        mesh.vertices = verts;
        mesh.uv = uvs;
        mesh.triangles = tris;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    static Vector3 SafeDir(Vector3 v, Vector3 fallback) =>
        v.sqrMagnitude < 1e-10f ? fallback : v.normalized;

    static Vector3 Perpendicular(Vector3 axis)
    {
        Vector3 reference = Mathf.Abs(Vector3.Dot(axis, Vector3.up)) > 0.99f
            ? Vector3.right : Vector3.up;
        return Vector3.Cross(axis, reference).normalized;
    }
}
