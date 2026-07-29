using UnityEngine;

/// <summary>
/// The load path of the winch line, solved analytically instead of through the
/// rigidbody chain.
///
/// WHY NOT JUST USE THE JOINT CHAIN
/// A winch line is the worst case for an iterative rigidbody solver. The links
/// weigh ~100 g each, the load weighs 1,200 kg, and the rope must be treated as
/// inextensible. That is a 12,000:1 mass ratio across a long serial chain of
/// hard constraints. PhysX cannot solve it: either the constraints go soft and
/// the rope silently stretches to twice its length while the drum reads almost
/// no tension, or they are stiffened with projection and mass scaling and the
/// solver injects energy until the load is flung away at 180 m/s. Both were
/// measured on this exact scene.
///
/// WHAT THIS DOES INSTEAD
/// The rope's load path is a one-dimensional elastic element, so it is solved
/// as one, using the rope's real material properties:
///
///     slack    = distance - freeLength
///     tension  = (EA / freeLength) * slack   +   c * closingRate
///     tension  = max(0, tension)                 (rope cannot push)
///
/// That is Hooke's law for a rod, with the same EA the rope spec reports. It is
/// exact for a taut rope, unconditionally stable, cannot inject energy, and
/// yields a clean tension signal for the winch drivetrain to load against.
///
/// The rigidbody chain in CableSimulator is still simulated -- it provides the
/// catenary sag, swing and collision that make the line look and behave right.
/// It simply no longer has to carry the load, which is the one job it was bad at.
/// </summary>
public class WinchRopeTether : MonoBehaviour
{
    [Header("Endpoints")]
    [Tooltip("Where the rope leaves the winch (fairlead or drum exit).")]
    public Transform ropeExit;

    [Tooltip("Where the rope attaches to the load.")]
    public Transform loadAttachment;

    [Tooltip("The body being pulled. If null, resolved from loadAttachment.")]
    public Rigidbody load;

    [Header("Rope")]
    public WireRopeSpec rope;

    [Tooltip("Free rope length between the two endpoints (m). Driven by the winch.")]
    public float freeLength = 4f;

    [Header("Elasticity")]
    [Tooltip("Fraction of the rope's true EA used for the tether spring. Leave " +
             "at 1: because this is a single spring against one solid load " +
             "rather than a chain of near-massless links, the rope's real " +
             "stiffness integrates perfectly well. At EA = 2.72 MN a 3/8\" rope " +
             "under 9 kN stretches 0.33% -- which is exactly what stress/E says " +
             "it should. Lower it only to model a deliberately springy line.")]
    [Range(0.001f, 1f)] public float stiffnessScale = 1f;

    [Tooltip("Largest fraction of a full oscillation the spring may complete in " +
             "one physics step. This is the actual stability criterion: a spring " +
             "is stable while omega*dt stays small. The tether automatically " +
             "softens itself against very light loads to respect it, so a real " +
             "stiffness can be used without ever exploding.")]
    [Range(0.02f, 0.5f)] public float maxStepPhase = 0.15f;

    [Tooltip("Damping ratio of the tether. Rope has real structural damping; " +
             "this also keeps the tether from ringing at its own frequency.")]
    [Range(0.05f, 2f)] public float dampingRatio = 0.7f;

    [Tooltip("Ceiling on tension as a multiple of the rope's breaking strength. " +
             "Stops a single bad frame from applying an impossible impulse.")]
    [Range(1f, 5f)] public float tensionClamp = 1.5f;

    [Header("Load Response")]
    [Tooltip("Apply the reaction force to the load. Turn off to let the winch " +
             "feel the load without the load feeling the winch (useful for " +
             "anchored recovery, where the winch pulls itself to the anchor).")]
    public bool applyForceToLoad = true;

    [Tooltip("Apply the reaction force to the winch's own body too, so an " +
             "unanchored winch gets dragged toward a heavier load.")]
    public bool applyForceToWinch = false;

    public Rigidbody winchBody;

    // ---- Live state ----

    /// <summary>Rope tension right now (N). Never negative -- rope cannot push.</summary>
    public float Tension { get; private set; }

    /// <summary>Straight-line distance between the endpoints (m).</summary>
    public float Distance { get; private set; }

    /// <summary>Extension beyond free length (m). Negative means slack.</summary>
    public float Slack => Distance - freeLength;

    /// <summary>True while the rope is carrying load.</summary>
    public bool IsTaut => Tension > 1f;

    /// <summary>Elongation as a fraction of free length.</summary>
    public float Strain => freeLength > 1e-4f
        ? Mathf.Max(0f, Slack) / freeLength : 0f;

    /// <summary>Tension as a fraction of the rope's breaking strength.</summary>
    public float LoadFraction => Spec != null && Spec.minimumBreakingStrength > 0f
        ? Tension / Spec.minimumBreakingStrength : 0f;

    /// <summary>Unit vector from the load toward the winch.</summary>
    public Vector3 PullDirection { get; private set; } = Vector3.forward;

    /// <summary>Rate the endpoints are separating (m/s). Positive = pulling apart.</summary>
    public float SeparationRate { get; private set; }

    public WireRopeSpec Spec => rope;

    /// <summary>Rope's true spring rate at the current free length (N/m).</summary>
    public float NominalSpringRate => rope != null
        ? rope.AxialStiffness * stiffnessScale / Mathf.Max(0.05f, freeLength)
        : 0f;

    /// <summary>
    /// Spring rate actually used this step (N/m), softened if the load is light
    /// enough that the rope's real stiffness could not be integrated stably.
    /// </summary>
    public float SpringRate { get; private set; }

    /// <summary>True when the tether had to soften itself for stability.</summary>
    public bool StiffnessLimited { get; private set; }

    /// <summary>
    /// Set by WinchController so it can call <see cref="Solve"/> itself in a
    /// defined order.
    ///
    /// Without this the tether and the winch both ran their own FixedUpdate and
    /// Unity's script execution order decided which went first. Going tether
    /// first, the drum loaded against tension computed from this step's rope
    /// length; going winch first, it loaded against last step's. One frame of
    /// lag in a feedback loop this stiff is not cosmetic -- it is what decides
    /// whether the loop damps or grows -- and nothing in the scene pinned the
    /// order, so it could differ between machines or after a reimport.
    /// </summary>
    [System.NonSerialized] public bool drivenExternally;

    void Reset()
    {
        winchBody = GetComponentInParent<Rigidbody>();
    }

    void Awake()
    {
        if (load == null && loadAttachment != null)
            load = loadAttachment.GetComponentInParent<Rigidbody>();
        if (winchBody == null) winchBody = GetComponentInParent<Rigidbody>();
    }

    /// <summary>Sets the free rope length. Called by the winch every step.</summary>
    public void SetFreeLength(float meters) =>
        freeLength = Mathf.Max(0.02f, meters);

    void FixedUpdate()
    {
        if (drivenExternally) return;   // WinchController calls Solve()
        Solve(Time.fixedDeltaTime);
    }

    /// <summary>Solve the load path for one step and apply the reaction forces.</summary>
    public void Solve(float dt)
    {
        Tension = 0f;

        if (ropeExit == null || loadAttachment == null || rope == null) return;
        if (load == null) load = loadAttachment.GetComponentInParent<Rigidbody>();

        Vector3 exit = ropeExit.position;
        Vector3 hook = loadAttachment.position;
        Vector3 delta = exit - hook;

        Distance = delta.magnitude;
        if (Distance < 1e-4f) return;

        PullDirection = delta / Distance;

        float extension = Distance - freeLength;

        // Relative velocity of the endpoints along the rope. Only the component
        // along the line matters; sideways motion swings the rope, it does not
        // stretch it.
        Vector3 vLoad = load != null ? load.GetPointVelocity(hook) : Vector3.zero;
        Vector3 vWinch = winchBody != null ? winchBody.GetPointVelocity(exit) : Vector3.zero;
        SeparationRate = Vector3.Dot(vLoad - vWinch, -PullDirection);

        if (extension <= 0f)
        {
            SeparationRate = 0f;
            return;   // slack rope carries nothing
        }

        // A spring-mass system integrates stably while omega*dt is small, with
        // omega = sqrt(k/m). Solving that for k gives the stiffest spring this
        // timestep can carry against this load. Against a 1,200 kg vehicle at a
        // 5 ms step that ceiling is around 4 MN/m, well above the rope's real
        // 0.3 MN/m, so the true stiffness is used untouched. Against something
        // very light the tether softens instead of exploding.
        float effectiveMass = load != null ? load.mass : 1f;
        float kMax = effectiveMass * (maxStepPhase / Mathf.Max(1e-4f, dt)) *
                                     (maxStepPhase / Mathf.Max(1e-4f, dt));

        float kNominal = NominalSpringRate;
        float k = Mathf.Min(kNominal, kMax);
        StiffnessLimited = k < kNominal * 0.999f;
        SpringRate = k;

        // Damping referenced to the mass actually being accelerated, so the
        // tether settles instead of ringing regardless of load size.
        float c = 2f * dampingRatio * Mathf.Sqrt(Mathf.Max(1e-4f, k * effectiveMass));

        float tension = k * extension + c * SeparationRate;

        // A rope pulls and never pushes, and cannot exceed its own strength by
        // more than the clamp allows.
        tension = Mathf.Clamp(tension, 0f,
                              rope.minimumBreakingStrength * tensionClamp);

        Tension = tension;

        if (applyForceToLoad && load != null)
            load.AddForceAtPosition(PullDirection * tension, hook, ForceMode.Force);

        if (applyForceToWinch && winchBody != null && !winchBody.isKinematic)
            winchBody.AddForceAtPosition(-PullDirection * tension, exit, ForceMode.Force);
    }

    void OnDrawGizmos()
    {
        if (ropeExit == null || loadAttachment == null) return;

        float f = Mathf.Clamp01(LoadFraction);
        Gizmos.color = IsTaut
            ? Color.Lerp(new Color(0.3f, 1f, 0.3f), new Color(1f, 0.2f, 0.1f), f)
            : new Color(0.5f, 0.5f, 0.5f, 0.4f);

        Gizmos.DrawLine(ropeExit.position, loadAttachment.position);
        Gizmos.DrawWireSphere(loadAttachment.position, 0.06f);
    }
}
