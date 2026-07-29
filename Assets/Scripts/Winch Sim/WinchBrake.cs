using UnityEngine;

/// <summary>
/// Automatic load-holding brake, the cone/dog brake fitted inside the drum of
/// every electric recovery winch.
///
/// Real behaviour this reproduces:
///  * It is normally ENGAGED. It releases only while the motor is powered.
///  * It only holds against PAYOUT. Powering in overruns it freely.
///  * It has a finite holding torque. Overload it and the drum creeps out.
///  * It is inside the drum, so it soaks up heat from every held load, and a
///    hot brake holds less. This is the documented cause of "winch slowly
///    letting the load down" on long holds.
///  * Releasing under load lets the load drop momentarily before the motor
///    catches it -- the lurch every operator knows.
/// </summary>
public class WinchBrake : MonoBehaviour
{
    [Header("Capacity")]
    [Tooltip("Static holding torque at the drum when cold (N*m).\n\n" +
             "Two cases size this, and the second is the harder one:\n\n" +
             "PARKED. It has to hold rated line pull on the first layer, since " +
             "that is the worst case the winch can create -- about 1,720 N*m " +
             "here, and never more than WinchMotor.StallDrumTorque.\n\n" +
             "POWERING OUT. It has to hold rated pull PLUS whatever the motor is " +
             "adding, at the reduced powerOutEngagement, because on the way down " +
             "the motor is helping the load rather than fighting it. That works " +
             "out near 1,910 N*m. 2,000 covers it cold with a little margin, so " +
             "a rated load powers down under control and only runs away once the " +
             "brake has actually faded -- which is the real failure mode, and the " +
             "reason winch manuals warn about long heavy power-outs.\n\n" +
             "The old 420 default held about 2,600 lbf and let any real recovery " +
             "load walk straight back out the moment power was cut.")]
    public float holdingTorque = 2000f;

    [Tooltip("Dynamic (sliding) torque as a fraction of static. Once a brake " +
             "starts slipping it holds less, which is why creep accelerates.")]
    [Range(0.3f, 1f)] public float dynamicFraction = 0.72f;

    [Tooltip("Hold only against payout, the way the cone/dog brake inside a winch " +
             "drum actually works -- powering in simply overruns it. Turn off to " +
             "model a symmetric disc brake.")]
    public bool oneWayHoldsPayoutOnly = true;

    [Header("Actuation")]
    [Tooltip("Time for the brake to travel to its power-out position (s).")]
    public float releaseTime = 0.12f;

    [Tooltip("Time for the brake to fully re-engage when power is cut (s).")]
    public float engageTime = 0.18f;

    [Tooltip("Engagement the brake settles at while the motor is powering out " +
             "(0-1).\n\n" +
             "A winch brake is not released when you power a load down, it is " +
             "MODULATED. The ramp inside it is unwound by the input shaft and " +
             "wound back up by the load, so the two balance and the load pays " +
             "out at whatever speed the motor is turning -- which is why " +
             "power-out speed barely changes with load, and why it is the brake " +
             "and not the motor that gets hot doing it.\n\n" +
             "Dropping to 0.9 over releaseTime is the momentary lurch you feel " +
             "when a load starts down before the drivetrain catches it. Set 1 " +
             "for no lurch.")]
    [Range(0.2f, 1f)] public float powerOutEngagement = 0.9f;

    [Header("Thermal")]
    [Tooltip("Thermal mass of the brake pack (J/K). It is small and buried " +
             "inside the drum, so it heats fast.")]
    public float thermalMass = 900f;

    [Tooltip("Heat rejection to ambient (W/K).")]
    public float coolingCoefficient = 2.2f;

    public float ambientTemperature = 20f;

    [Tooltip("Temperature at which holding torque starts to fade (C).")]
    public float fadeStartTemp = 150f;

    [Tooltip("Temperature at which the brake retains only residual grip (C).")]
    public float fadeEndTemp = 320f;

    [Tooltip("Fraction of holding torque still available when fully faded.")]
    [Range(0f, 0.6f)] public float residualFraction = 0.25f;

    // ---- State ----

    /// <summary>0 = fully released, 1 = fully engaged.</summary>
    public float Engagement { get; private set; } = 1f;

    public float Temperature { get; private set; }

    /// <summary>True when the brake is slipping under load rather than holding.</summary>
    public bool IsSlipping { get; private set; }

    /// <summary>Torque the brake actually applied last step (N*m).</summary>
    public float AppliedTorque { get; private set; }

    /// <summary>0-1 fade factor from brake temperature.</summary>
    public float FadeFactor
    {
        get
        {
            if (Temperature <= fadeStartTemp) return 1f;
            if (Temperature >= fadeEndTemp) return residualFraction;
            float t = (Temperature - fadeStartTemp) / (fadeEndTemp - fadeStartTemp);
            return Mathf.Lerp(1f, residualFraction, t);
        }
    }

    /// <summary>Static holding torque available right now (N*m, &gt;= 0).</summary>
    public float StaticCapacity =>
        Mathf.Max(0f, holdingTorque * FadeFactor * Engagement);

    /// <summary>Sliding torque once it has broken loose (N*m, &gt;= 0).</summary>
    public float DynamicCapacity => StaticCapacity * dynamicFraction;

    /// <summary>Alias kept for readouts.</summary>
    public float AvailableTorque => StaticCapacity;

    void Awake() => Temperature = ambientTemperature;

    /// <summary>
    /// Advance the actuator and cool the pack. Call once per step, before the
    /// drivetrain solve.
    ///
    /// The brake no longer decides its own torque. It cannot: a brake is dry
    /// friction, and dry friction is a constraint on velocity, not a torque you
    /// can integrate. The old version returned sign(omega) * capacity, which
    /// over one 5 ms step is far more velocity change than the drum has, so it
    /// overshot zero and reversed sign every step -- a held load buzzed rather
    /// than sat still. WinchController now solves brake and drivetrain friction
    /// together against the drum's actual momentum, and reports the result back
    /// through <see cref="ReportApplied"/>.
    /// </summary>
    /// <param name="poweringOut">True while the motor is driving in the payout
    /// direction. Powering IN is not a release: the brake is one-way and is
    /// simply overrun, so it stays where it is.</param>
    /// <param name="clutchOut">True in freespool, where the brake is
    /// mechanically out of the loop entirely.</param>
    public void UpdateActuation(bool poweringOut, bool clutchOut, float dt)
    {
        float target = clutchOut ? 0f : (poweringOut ? powerOutEngagement : 1f);
        bool releasing = target < Engagement;

        float rate = releasing ? 1f / Mathf.Max(1e-3f, releaseTime)
                               : 1f / Mathf.Max(1e-3f, engageTime);
        Engagement = Mathf.MoveTowards(Engagement, target, rate * dt);
    }

    /// <summary>
    /// Record the torque the drivetrain solver actually took from the brake, so
    /// heat and state match the constraint that was really applied instead of
    /// the one the brake guessed at.
    /// </summary>
    /// <param name="torque">Signed torque the solver credited to the brake (N*m).</param>
    /// <param name="slipOmega">Drum speed it was taken at (rad/s).</param>
    /// <param name="slipping">True if the brake was overrun rather than holding
    /// what it was asked to hold. Reported state only -- heat does not depend
    /// on it.</param>
    public void ReportApplied(float torque, float slipOmega, bool slipping, float dt)
    {
        AppliedTorque = torque;
        IsSlipping = slipping;

        // Heat is friction work: torque times the speed it was applied at. Not
        // gated on the slipping flag, because a brake modulating a load down is
        // holding it and still burning every watt of it. That case -- powering a
        // heavy load out -- is the one that actually cooks a winch brake, and
        // gating heat on "slipping" was exactly why the sim never saw it.
        StepThermal(Mathf.Abs(torque * slipOmega), dt);
    }

    /// <summary>Line pull this brake can hold at a given drum radius (N).</summary>
    public float HoldableLinePull(float effectiveRadius) =>
        StaticCapacity / Mathf.Max(1e-4f, effectiveRadius);

    void StepThermal(float heatWatts, float dt)
    {
        float cooling = coolingCoefficient * (Temperature - ambientTemperature);
        Temperature += (heatWatts - cooling) / Mathf.Max(1f, thermalMass) * dt;
        Temperature = Mathf.Max(ambientTemperature, Temperature);
    }

    public void ResetThermal() => Temperature = ambientTemperature;
}
