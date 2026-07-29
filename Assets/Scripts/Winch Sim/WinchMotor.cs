using UnityEngine;

/// <summary>
/// Series-wound 12V DC motor with a planetary gearbox -- the drivetrain used by
/// essentially every electric recovery winch.
///
/// WHY SERIES-WOUND MATTERS
/// A permanent-magnet motor has constant flux, so T = kt*I and speed falls
/// linearly with torque. Fitting that model to a real winch is impossible: the
/// kt that reproduces 9,500 lb of rated pull predicts a free speed of about
/// 3,500 rpm, but the same winch actually free-spins near 7,000 rpm.
///
/// A series motor puts the field winding in series with the armature, so field
/// flux rises with current:
///
///     phi(I) = kPhi * I / (1 + I / Isat)     (saturating; iron cannot hold
///                                             unlimited flux)
///     T      = phi(I) * I                    (roughly I^2 below saturation)
///     E_back = phi(I) * omega
///     V      = I*R + E_back
///
/// That single change reproduces BOTH published operating points of a real
/// winch at once, and it is the reason a winch has enormous stall torque yet
/// still runs fast with no load. It is also why line speed collapses so
/// dramatically the moment a load comes on -- the behaviour operators describe
/// as the winch "loading up".
///
/// Default constants are fitted to a Warn 9.5xp class unit:
///   rated 9,500 lbf @ 435 A, 11.4 fpm       no-load 70 A, 35.7 fpm
/// Fitted result: line speed within 5% and current within 5% across the
/// published 0-6,000 lb band, and the per-layer max-pull curve tracks the
/// published table to within 1% of a constant 1.12 stall-over-rated margin.
/// </summary>
public class WinchMotor : MonoBehaviour
{
    [Header("Electrical")]
    [Tooltip("Battery open-circuit voltage (V). 12.8 = healthy resting AGM.")]
    public float nominalVoltage = 12.8f;

    [Tooltip("Armature + field winding resistance (ohm).")]
    public float motorResistance = 0.0109f;

    [Tooltip("Battery internal resistance + cable resistance (ohm). " +
             "Healthy AGM on 2 AWG leads ~0.002. Raise it to simulate a tired " +
             "battery or undersized cable and watch the winch lose pull.")]
    public float supplyResistance = 0.002f;

    [Tooltip("Field flux constant (Wb/A). Sets torque per amp before saturation.")]
    public float fluxConstant = 3.60e-4f;

    [Tooltip("Current at which the field iron is half saturated (A). Below this " +
             "torque grows as I^2; above it, torque grows only linearly.")]
    public float saturationCurrent = 110f;

    [Tooltip("Maximum current the solenoid pack and cabling will pass (A). " +
             "Also the stall current. 9.5k class winch: 430-480 A.")]
    public float maxCurrent = 420f;

    [Header("Gearbox")]
    [Tooltip("Total gear reduction. 3-stage planetary, Warn 9.5xp: 156:1.")]
    public float gearRatio = 156f;

    [Tooltip("Mechanical efficiency of the gear train under load (0-1). " +
             "3-stage planetary, fitted to published data: 0.90. A worm-drive " +
             "winch would be ~0.35, which is why they are slow but self-locking.")]
    [Range(0.1f, 1f)] public float gearEfficiency = 0.90f;

    [Tooltip("Rotational inertia of the armature (kg*m^2). Tiny, but it is " +
             "multiplied by gearRatio^2 when reflected to the drum, where it " +
             "dominates the drum's own inertia by orders of magnitude.")]
    public float armatureInertia = 6e-5f;

    [Tooltip("Highest credible armature speed (rpm). This winch free-spins near " +
             "7,000, so 10,000 is comfortably past anything it does in service.\n\n" +
             "It exists because an overhauling load has nothing else stopping it. " +
             "A series motor makes no torque above its free speed and cannot " +
             "usefully regenerate, so if the brake ever fades out from under a " +
             "heavy load the only thing left resisting is viscous drag -- and " +
             "solving that balance puts the drum at 21 m/s and the armature at " +
             "65,000 rpm. No real drivetrain reaches that; it disassembles first. " +
             "Rather than extrapolate a fitted motor curve into a state the " +
             "hardware cannot occupy, the drivetrain clamps here and flags " +
             "Overspeed so the caller knows the model has left its valid range.")]
    public float maxArmatureRpm = 10000f;

    /// <summary>Drum speed corresponding to <see cref="maxArmatureRpm"/> (rad/s).</summary>
    public float MaxDrumSpeed =>
        maxArmatureRpm * 2f * Mathf.PI / 60f / Mathf.Max(1f, gearRatio);

    [Header("Drivetrain Friction (referred to the drum)")]
    [Tooltip("Coulomb friction torque at the drum (N*m). This is large and " +
             "deliberately so: it is what the published 70 A no-load current " +
             "actually represents, and it is why a winch does not backdrive.")]
    public float coulombFriction = 135f;

    [Tooltip("Viscous friction at the drum (N*m per rad/s).")]
    public float viscousFriction = 3f;

    [Tooltip("Extra friction per unit of rope torque, dimensionless. Gear teeth " +
             "and bearings lose more the harder they are loaded, which is a real " +
             "effect and the reason a loaded winch resists backdriving better " +
             "than an unloaded one.\n\n" +
             "Ships at 0, and should stay there for the shipped Warn fit: " +
             "coulombFriction and gearEfficiency were fitted to published data, " +
             "so the load-dependent share of the real winch's losses is already " +
             "inside those numbers. Turning this up double-counts them and the " +
             "sim loses pull it should have. Raise it only when modelling a " +
             "drivetrain you are fitting from scratch, or a worm drive.")]
    [Range(0f, 0.4f)] public float loadDependentFriction = 0f;

    [Header("Thermal")]
    [Tooltip("Thermal mass of the motor (J/K).")]
    public float thermalMass = 4200f;

    [Tooltip("Heat rejection to ambient (W/K).")]
    public float coolingCoefficient = 5.5f;

    public float ambientTemperature = 20f;

    [Tooltip("Temperature at which output starts derating (C).")]
    public float derateStartTemp = 120f;

    [Tooltip("Temperature at which the motor cuts out entirely (C). " +
             "This, not rated line pull, is the real limit on a long pull.")]
    public float thermalCutoffTemp = 180f;

    // ---- Live state ----
    public float Temperature { get; private set; }
    public float Current { get; private set; }
    public float TerminalVoltage { get; private set; }
    public float ShaftTorque { get; private set; }
    public float ElectricalPower { get; private set; }
    public float MechanicalPower { get; private set; }
    public bool ThermalCutout { get; private set; }

    /// <summary>Inertia the armature presents at the drum shaft (kg*m^2).</summary>
    public float ReflectedInertia => armatureInertia * gearRatio * gearRatio;

    float CircuitResistance => Mathf.Max(1e-4f, motorResistance + supplyResistance);

    /// <summary>Saturating field flux at a given current (Wb).</summary>
    public float Flux(float amps)
    {
        float a = Mathf.Abs(amps);
        return fluxConstant * a / (1f + a / Mathf.Max(1f, saturationCurrent));
    }

    /// <summary>0-1 derate factor from motor temperature.</summary>
    public float ThermalDerate
    {
        get
        {
            if (Temperature <= derateStartTemp) return 1f;
            if (Temperature >= thermalCutoffTemp) return 0f;
            return 1f - (Temperature - derateStartTemp) /
                        (thermalCutoffTemp - derateStartTemp);
        }
    }

    public float Efficiency => ElectricalPower > 1f
        ? Mathf.Clamp01(MechanicalPower / ElectricalPower) : 0f;

    void Awake() => Temperature = ambientTemperature;

    /// <summary>
    /// Solve the motor for one step.
    /// </summary>
    /// <param name="duty">-1..1 commanded direction. Contactor winches are
    /// on/off, so this is normally -1, 0 or 1.</param>
    /// <param name="drumOmega">Drum angular velocity (rad/s), signed.</param>
    /// <param name="dt">Timestep (s).</param>
    /// <returns>Torque delivered at the DRUM shaft (N*m), signed.</returns>
    public float Step(float duty, float drumOmega, float dt)
    {
        duty = Mathf.Clamp(duty, -1f, 1f);

        // Thermal cutout latches until the motor cools 20 C below the limit.
        if (Temperature >= thermalCutoffTemp) ThermalCutout = true;
        else if (Temperature < thermalCutoffTemp - 20f) ThermalCutout = false;
        if (ThermalCutout) duty = 0f;

        if (Mathf.Abs(duty) < 0.01f)
        {
            Current = 0f;
            ShaftTorque = 0f;
            TerminalVoltage = nominalVoltage;
            ElectricalPower = 0f;
            MechanicalPower = 0f;
            StepThermal(0f, dt);
            return 0f;
        }

        // Motor shaft speed in the direction the motor is driving. A series
        // motor always produces torque in the driven direction regardless of
        // rotation, so we work in the driving frame and sign the result at the end.
        float motorOmega = drumOmega * gearRatio * Mathf.Sign(duty);

        float current = SolveCurrent(motorOmega);
        float phi = Flux(current);

        float motorTorque = phi * current * ThermalDerate;

        TerminalVoltage = nominalVoltage - current * supplyResistance;
        float drumTorque = motorTorque * gearRatio * gearEfficiency * Mathf.Sign(duty);

        Current = current;
        ShaftTorque = motorTorque;
        ElectricalPower = TerminalVoltage * current;
        MechanicalPower = Mathf.Abs(drumTorque * drumOmega);

        // Resistive heating dominates; add the gearbox loss.
        float heat = current * current * motorResistance
                   + Mathf.Abs(motorTorque * motorOmega) * (1f - gearEfficiency);
        StepThermal(heat, dt);

        return drumTorque;
    }

    /// <summary>
    /// Solve V = I*R + phi(I)*omega for I.
    ///
    /// With the saturating flux law this is a quadratic in I, so it has a
    /// closed form and needs no iteration:
    ///
    ///   V = I*R + kPhi*I/(1 + I/Is) * w
    ///
    /// Multiply through by (1 + I/Is):
    ///
    ///   V + V*I/Is = I*R + R*I^2/Is + kPhi*I*w
    ///
    /// which rearranges to
    ///
    ///   (R/Is) I^2 + (R + kPhi*w - V/Is) I - V = 0
    ///
    /// Sanity check: at w = 0 this returns I = V/R, the true stall current.
    /// </summary>
    float SolveCurrent(float motorOmega)
    {
        float R = CircuitResistance;
        float Is = Mathf.Max(1f, saturationCurrent);
        float w = Mathf.Max(0f, motorOmega);   // regenerating is handled by friction

        float a = R / Is;
        float b = R + fluxConstant * w - nominalVoltage / Is;
        float c = -nominalVoltage;

        float disc = b * b - 4f * a * c;
        float i = (-b + Mathf.Sqrt(Mathf.Max(0f, disc))) / (2f * a);

        return Mathf.Clamp(i, 0f, maxCurrent);
    }

    void StepThermal(float heatWatts, float dt)
    {
        float cooling = coolingCoefficient * (Temperature - ambientTemperature);
        Temperature += (heatWatts - cooling) / Mathf.Max(1f, thermalMass) * dt;
        Temperature = Mathf.Max(ambientTemperature, Temperature);
    }

    /// <summary>
    /// Total friction torque opposing drum motion (N*m), signed against
    /// <paramref name="drumOmega"/>.
    ///
    /// DEPRECATED for use inside the integrator. Dry friction cannot be
    /// integrated as a torque: 135 N*m against ~1.5 kg*m^2 of reflected inertia
    /// is a 0.45 rad/s velocity change per 5 ms step, an order of magnitude
    /// wider than any sign-smoothing band, so sign(omega) flips every step and
    /// the drum buzzes instead of sitting still. WinchController applies
    /// <see cref="CoulombFrictionAt"/> as a velocity constraint instead.
    /// This remains only for readouts and external tooling.
    /// </summary>
    public float FrictionTorque(float drumOmega)
    {
        float sign = Mathf.Clamp(drumOmega / 0.05f, -1f, 1f);
        return -(coulombFriction * sign + viscousFriction * drumOmega);
    }

    /// <summary>
    /// Coulomb friction magnitude at the drum (N*m, always positive) for a given
    /// rope torque. Unsigned on purpose: the caller decides which way it acts,
    /// and it must be able to hold at exactly zero speed.
    /// </summary>
    public float CoulombFrictionAt(float ropeTorque) =>
        Mathf.Max(0f, coulombFriction) +
        loadDependentFriction * Mathf.Abs(ropeTorque);

    /// <summary>Viscous friction coefficient at the drum (N*m per rad/s).</summary>
    public float ViscousCoefficient => Mathf.Max(0f, viscousFriction);

    /// <summary>
    /// Drum torque this motor would make at a given drum speed, without
    /// touching any state. Pure: no current, no heat, no cutout latching.
    ///
    /// This exists so the drivetrain integrator can evaluate the torque curve at
    /// speeds the drum has not reached yet, which is what solving the step
    /// implicitly requires. It has to be the exact curve rather than a local
    /// slope, because the curve is not locally informative: below about 1.3
    /// rad/s at the drum the motor is hard against its current limit, so the
    /// slope there is exactly zero while the torque a fraction of a rad/s later
    /// is already collapsing. Anything that linearises about the current point
    /// sees a flat curve and steps straight off the end of it.
    /// </summary>
    /// <param name="duty">-1..1 commanded direction.</param>
    /// <param name="drumOmega">Drum angular velocity to evaluate at (rad/s).</param>
    /// <returns>Torque at the DRUM shaft (N*m), signed by duty.</returns>
    public float DrumTorqueAt(float duty, float drumOmega)
    {
        if (Mathf.Abs(duty) < 0.01f || ThermalCutout) return 0f;

        float s = Mathf.Sign(duty);
        float motorOmega = drumOmega * gearRatio * s;

        float i = SolveCurrent(motorOmega);
        float motorTorque = Flux(i) * i * ThermalDerate;

        return motorTorque * gearRatio * gearEfficiency * s;
    }

    /// <summary>Stall torque at the drum (N*m), net of drivetrain friction.</summary>
    public float StallDrumTorque
    {
        get
        {
            float i = Mathf.Min(maxCurrent, nominalVoltage / CircuitResistance);
            float t = Flux(i) * i * ThermalDerate * gearRatio * gearEfficiency;
            return Mathf.Max(0f, t - coulombFriction);
        }
    }

    /// <summary>Free-running drum speed with no load (rad/s), solved from the
    /// point where motor torque exactly balances drivetrain friction.</summary>
    public float NoLoadDrumSpeed
    {
        get
        {
            float lo = 0f, hi = 2000f;   // motor rad/s
            for (int k = 0; k < 40; k++)
            {
                float mid = (lo + hi) * 0.5f;
                float i = SolveCurrent(mid);
                float drumT = Flux(i) * i * gearRatio * gearEfficiency;
                float need = coulombFriction + viscousFriction * (mid / gearRatio);
                if (drumT > need) lo = mid; else hi = mid;
            }
            return lo / gearRatio;
        }
    }

    void OnValidate()
    {
        gearRatio = Mathf.Max(1f, gearRatio);
        motorResistance = Mathf.Max(1e-4f, motorResistance);
        supplyResistance = Mathf.Max(0f, supplyResistance);
        fluxConstant = Mathf.Max(1e-6f, fluxConstant);
        saturationCurrent = Mathf.Max(1f, saturationCurrent);
    }
}
