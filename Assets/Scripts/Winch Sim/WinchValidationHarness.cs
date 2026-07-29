using System.Collections;
using System.Text;
using UnityEngine;

/// <summary>
/// Drives the winch through a scripted duty cycle in play mode and records the
/// telemetry, so the sim can be checked against a real winch spec sheet instead
/// of judged by eye.
///
/// This is a validation tool, not part of the simulation.
/// </summary>
public class WinchValidationHarness : MonoBehaviour
{
    public WinchController winch;
    public Rigidbody load;
    public float sampleInterval = 0.5f;

    [Tooltip("Disable rope failure during the run so the drivetrain can be " +
             "measured without the rope parting and ending the test early.")]
    public bool suppressRopeFailure = true;

    public bool Done { get; private set; }
    public string Report { get; private set; } = "";

    private float _command;
    private readonly StringBuilder _sb = new StringBuilder();

    public void Begin()
    {
        Done = false;
        _command = 0f;
        _sb.Length = 0;
        StopAllCoroutines();
        StartCoroutine(Run());
    }

    void FixedUpdate()
    {
        if (winch != null) winch.SetCommand(_command);
    }

    IEnumerator Run()
    {
        var rope = winch.rope;
        if (suppressRopeFailure && rope != null) rope.simulateRopeFailure = false;
        if (load == null && rope != null && rope.endAnchor != null)
            load = rope.endAnchor.GetComponentInParent<Rigidbody>();

        _sb.AppendLine("   t | phase | state      | dep  | lyr | drumT lbf | loadT lbf |   fpm | amps |   V  | span | len  | strch | load v | nodes");

        yield return Phase("settle", 0f, 1.5f);
        yield return Phase("IN", 1f, 9.0f);
        yield return Phase("hold", 0f, 2.0f);
        yield return Phase("OUT", -1f, 3.0f);
        yield return Phase("hold", 0f, 1.5f);

        Report = _sb.ToString();
        Done = true;
    }

    IEnumerator Phase(string name, float cmd, float duration)
    {
        _command = cmd;
        float t = 0f, next = 0f;
        while (t < duration)
        {
            if (t >= next) { Sample(name, t); next += sampleInterval; }
            t += Time.deltaTime;
            yield return null;
        }
    }

    void Sample(string phase, float t)
    {
        var rope = winch.rope;
        var drum = winch.drum;

        const float N_TO_LBF = 0.2248089f;
        float stretch = rope != null && rope.cableLength > 1e-3f
            ? rope.MeasuredLength / rope.cableLength : 0f;

        _sb.AppendLine(
            t.ToString("F2").PadLeft(5) + " | " +
            phase.PadRight(5) + " | " +
            winch.State.ToString().PadRight(10) + " | " +
            drum.DeployedLength.ToString("F2").PadLeft(4) + " | " +
            (drum.CurrentLayer + 1).ToString().PadLeft(3) + " | " +
            (rope != null ? (rope.TensionAtStart * N_TO_LBF).ToString("N0") : "-").PadLeft(9) + " | " +
            (rope != null ? (rope.TensionAtEnd * N_TO_LBF).ToString("N0") : "-").PadLeft(9) + " | " +
            winch.LineSpeedFpm.ToString("F1").PadLeft(5) + " | " +
            winch.MotorCurrent.ToString("F0").PadLeft(4) + " | " +
            winch.BatteryVoltage.ToString("F1").PadLeft(4) + " | " +
            (rope != null ? rope.SpanDistance.ToString("F2") : "-").PadLeft(4) + " | " +
            (rope != null ? rope.cableLength.ToString("F2") : "-").PadLeft(4) + " | " +
            stretch.ToString("F2").PadLeft(5) + " | " +
            (load != null ? load.linearVelocity.magnitude.ToString("F2") : "-").PadLeft(6) + " | " +
            (rope != null ? rope.NodeCount.ToString() : "-").PadLeft(5));
    }
}
