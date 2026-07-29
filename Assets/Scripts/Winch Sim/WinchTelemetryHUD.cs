using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// On-screen readout of everything the winch drivetrain is doing. IMGUI so it
/// needs no scene setup, no Canvas and no prefab wiring -- drop it on the winch
/// and it works.
///
/// The values shown are the ones on a real winch spec sheet, so the sim can be
/// checked against published data directly.
/// </summary>
public class WinchTelemetryHUD : MonoBehaviour
{
    public WinchController winch;
    public CableSimulator rope;

    [Header("Display")]
    public bool show = true;
    public Key toggleKey = Key.F1;
    public bool imperialUnits = true;
    public Vector2 origin = new Vector2(12f, 12f);
    public float width = 330f;
    [Range(9, 24)] public int fontSize = 12;

    private GUIStyle _label, _header, _box, _bar;
    private Texture2D _white;

    void Awake()
    {
        if (winch == null) winch = GetComponent<WinchController>();
        if (rope == null && winch != null) rope = winch.rope;

        _white = new Texture2D(1, 1);
        _white.SetPixel(0, 0, Color.white);
        _white.Apply();
    }

    void Update()
    {
        var kb = Keyboard.current;
        if (kb != null && kb[toggleKey].wasPressedThisFrame) show = !show;
    }

    void EnsureStyles()
    {
        if (_label != null) return;

        _label = new GUIStyle(GUI.skin.label)
        {
            fontSize = fontSize,
            richText = true,
            padding = new RectOffset(2, 2, 1, 1)
        };
        _header = new GUIStyle(_label) { fontStyle = FontStyle.Bold };
        _box = new GUIStyle(GUI.skin.box) { padding = new RectOffset(10, 10, 8, 8) };
        _bar = new GUIStyle();
    }

    void OnGUI()
    {
        if (!show || winch == null) return;
        EnsureStyles();

        var drum = winch.drum;
        var motor = winch.motor;
        var brake = winch.brake;

        float height = 396f;
        GUILayout.BeginArea(new Rect(origin.x, origin.y, width, height), GUIContent.none, _box);

        GUILayout.Label($"<b>WINCH</b>   <color=#7fd>{winch.State}</color>", _header);
        Separator();

        // --- Line ---
        GUILayout.Label("<b>LINE</b>", _header);
        if (imperialUnits)
        {
            Row("Line pull", $"{winch.LinePullLbf:N0} lbf");
            Row("Available pull", $"{winch.AvailableLinePull * 0.2248089f:N0} lbf");
            Row("Line speed", $"{winch.LineSpeedFpm:N1} fpm");
        }
        else
        {
            Row("Line pull", $"{winch.LinePull / 1000f:N2} kN");
            Row("Available pull", $"{winch.AvailableLinePull / 1000f:N2} kN");
            Row("Line speed", $"{winch.LineSpeed:N3} m/s");
        }
        Row("Rope power", $"{winch.RopePower:N0} W");

        if (rope != null)
        {
            float frac = rope.LoadFraction;
            Row("Tension / MBS", $"{frac:P0}", TensionColour(frac));
            Bar(frac, TensionColour(frac));
            Row("Slack", rope.IsSlack ? $"yes ({rope.Tautness:P0} taut)" : "no");
            if (rope.HasFailed)
                GUILayout.Label("<color=#f55><b>ROPE PARTED</b></color>", _header);
        }

        Separator();

        // --- Drum ---
        GUILayout.Label("<b>DRUM</b>", _header);
        if (drum != null)
        {
            Row("Layer", $"{drum.CurrentLayer + 1} of {drum.LayerCount}");
            Row("Eff. radius", $"{drum.EffectiveRadius * 1000f:N1} mm");
            if (imperialUnits)
                Row("Deployed", $"{drum.DeployedLength * 3.28084f:N1} ft " +
                                $"of {drum.totalRopeLength * 3.28084f:N0} ft");
            else
                Row("Deployed", $"{drum.DeployedLength:N2} m of {drum.totalRopeLength:N1} m");
            Bar(drum.PayoutFraction, new Color(0.4f, 0.7f, 1f));

            Row("Wraps left", $"{drum.WrapsRemaining:N1}",
                drum.AtMinimumWraps ? new Color(1f, 0.4f, 0.3f) : Color.white);
            Row("Drum speed", $"{winch.DrumOmega * 9.5493f:N1} rpm");
            Row("Inertia", $"{drum.TotalInertia:N4} kg·m²");

            if (drum.ViolatesMinBendRadius)
                GUILayout.Label("<color=#fa4>Barrel below rope min bend radius</color>", _label);
        }

        Separator();

        // --- Motor ---
        GUILayout.Label("<b>MOTOR</b>", _header);
        if (motor != null)
        {
            Row("Current", $"{motor.Current:N0} A", CurrentColour(motor.Current));
            Bar(Mathf.Clamp01(motor.Current / Mathf.Max(1f, motor.maxCurrent)),
                CurrentColour(motor.Current));
            Row("Battery", $"{motor.TerminalVoltage:N2} V",
                motor.TerminalVoltage < 10.5f ? new Color(1f, 0.5f, 0.3f) : Color.white);
            Row("Drum torque", $"{winch.MotorTorque:N0} N·m");
            Row("Efficiency", $"{motor.Efficiency:P0}");
            Row("Motor temp", $"{motor.Temperature:N0} °C", TempColour(
                motor.Temperature, motor.derateStartTemp, motor.thermalCutoffTemp));
            if (motor.ThermalDerate < 1f)
                Row("Thermal derate", $"{motor.ThermalDerate:P0}", new Color(1f, 0.6f, 0.2f));
            if (motor.ThermalCutout)
                GUILayout.Label("<color=#f55><b>THERMAL CUTOUT</b></color>", _header);
        }

        Separator();

        // --- Brake ---
        GUILayout.Label("<b>BRAKE</b>", _header);
        if (brake != null)
        {
            Row("Engagement", $"{brake.Engagement:P0}");
            Row("Torque", $"{brake.AppliedTorque:N0} N·m");
            Row("Temp", $"{brake.Temperature:N0} °C",
                TempColour(brake.Temperature, brake.fadeStartTemp, brake.fadeEndTemp));
            if (brake.FadeFactor < 1f)
                Row("Fade", $"{brake.FadeFactor:P0} held", new Color(1f, 0.6f, 0.2f));
            if (brake.IsSlipping && Mathf.Abs(winch.DrumOmega) > 0.02f && winch.State == WinchController.WinchState.Idle)
                GUILayout.Label("<color=#f84><b>BRAKE CREEPING</b></color>", _header);
        }

        GUILayout.FlexibleSpace();
        GUILayout.Label($"<color=#888>↑/↓ spool  •  Shift freespool  •  {toggleKey} hide</color>", _label);

        GUILayout.EndArea();
    }

    void Row(string k, string v) => Row(k, v, Color.white);

    void Row(string k, string v, Color c)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label($"<color=#aaa>{k}</color>", _label, GUILayout.Width(width * 0.42f));
        string hex = ColorUtility.ToHtmlStringRGB(c);
        GUILayout.Label($"<color=#{hex}>{v}</color>", _label);
        GUILayout.EndHorizontal();
    }

    void Bar(float fraction01, Color c)
    {
        Rect r = GUILayoutUtility.GetRect(width - 24f, 5f);
        GUI.color = new Color(1f, 1f, 1f, 0.12f);
        GUI.DrawTexture(r, _white);
        GUI.color = c;
        GUI.DrawTexture(new Rect(r.x, r.y, r.width * Mathf.Clamp01(fraction01), r.height), _white);
        GUI.color = Color.white;
    }

    void Separator()
    {
        Rect r = GUILayoutUtility.GetRect(width - 24f, 6f);
        GUI.color = new Color(1f, 1f, 1f, 0.15f);
        GUI.DrawTexture(new Rect(r.x, r.y + 3f, r.width, 1f), _white);
        GUI.color = Color.white;
    }

    static Color TensionColour(float frac)
    {
        if (frac < 0.4f) return new Color(0.5f, 1f, 0.5f);
        if (frac < 0.7f) return new Color(1f, 0.9f, 0.4f);
        return new Color(1f, 0.35f, 0.3f);
    }

    static Color CurrentColour(float amps)
    {
        if (amps < 150f) return new Color(0.5f, 1f, 0.6f);
        if (amps < 380f) return new Color(1f, 0.9f, 0.4f);
        return new Color(1f, 0.35f, 0.3f);
    }

    static Color TempColour(float t, float warn, float limit)
    {
        if (t < warn) return Color.white;
        if (t < limit) return new Color(1f, 0.7f, 0.3f);
        return new Color(1f, 0.35f, 0.3f);
    }
}
