using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// The winch drivetrain: solves the rotational equation of motion for the drum
/// and drives the rope from the result.
///
///     I(t) * dOmega/dt = T_motor + T_brake + T_friction - T_rope
///
/// with
///     I(t)        drum + spooled rope + gear-reflected armature inertia
///     T_motor     from WinchMotor, speed and temperature dependent
///     T_brake     from WinchBrake, load-holding and fade dependent
///     T_rope      rope tension * effective spool radius
///
/// Nothing here commands a speed. Speed is whatever the torque balance produces,
/// which is the entire point: a heavily loaded winch is slow because the motor
/// cannot make more torque, not because a script told it to be slow.
///
/// Modes:
///   Powered in / out   contactor closed, brake released, motor drives
///   Holding            contactor open, brake engaged, load held (or creeping)
///   Freespool          clutch out, drum disconnected from the gearbox entirely
/// </summary>
[RequireComponent(typeof(WinchDrum))]
public class WinchController : MonoBehaviour
{
    public enum WinchState { Idle, PowerIn, PowerOut, Freespool, ThermalCutout, Stalled }

    [Header("Drivetrain")]
    public WinchDrum drum;
    public WinchMotor motor;
    public WinchBrake brake;

    [Header("Rope")]
    [Tooltip("Rope this winch spools. Its length is driven by the drum. This is " +
             "the visual/shape simulation.")]
    public CableSimulator rope;

    [Tooltip("Analytic load path. This is what actually transmits force between " +
             "the winch and the load, and what the drivetrain loads against. " +
             "See WinchRopeTether for why the rigidbody chain does not do this.")]
    public WinchRopeTether tether;

    [Tooltip("Extra rope kept outside the drum so the sim never runs out of " +
             "nodes at full retraction (m).")]
    public float minimumFreeLength = 0.35f;

    [Header("Visual")]
    [Tooltip("Transform of the visible drum. Rotated by the solved angle.")]
    public Transform drumVisual;

    [Tooltip("Local axis the visible drum spins about.")]
    public Vector3 drumSpinAxis = Vector3.forward;

    [Header("Freespool")]
    [Tooltip("Allow the clutch to be disengaged. In freespool the drum is " +
             "disconnected from the gearbox and the rope can be pulled out by hand.")]
    public bool allowFreespool = true;

    [Tooltip("Drag on the drum in freespool (N*m per rad/s). Real drums have " +
             "just enough to prevent birdnesting -- but not much.")]
    public float freespoolDrag = 0.12f;

    [Header("Safety Limits")]
    [Tooltip("Stop powering in when the rope is fully retracted.")]
    public bool stopAtFullRetract = true;

    [Tooltip("Refuse to pay out past the minimum-wrap reserve.")]
    public bool enforceMinimumWraps = true;

    [Header("Input")]
    public InputActionAsset inputActions;
    public string actionMapName = "Winch";
    public string moveActionName = "Move";
    public string freespoolActionName = "Freespool";

    [Tooltip("Keyboard fallback used when no InputActionAsset is assigned.")]
    public bool useKeyboardFallback = true;

    [Tooltip("Ignore all input devices and take the command from SetCommand() " +
             "instead. Use this to drive the winch from a remote, an AI, a test " +
             "harness or a replay.")]
    public bool externalControl = false;

    // ---- Live state ----

    /// <summary>Drum angular velocity, rad/s. Positive = spooling in.</summary>
    public float DrumOmega { get; private set; }

    /// <summary>Accumulated drum angle, rad.</summary>
    public float DrumAngle { get; private set; }

    /// <summary>True when an overhauling load has driven the drum to the
    /// drivetrain's speed limit. The brake has faded or is undersized, and
    /// readings past this point are outside the model's valid range.</summary>
    public bool Overspeed { get; private set; }

    public WinchState State { get; private set; } = WinchState.Idle;
    public bool ClutchEngaged { get; private set; } = true;

    /// <summary>Rope tension at the drum this step (N).</summary>
    public float RopeTension { get; private set; }

    /// <summary>Force the rope is being pulled with (N). Same as tension, named
    /// the way spec sheets name it.</summary>
    public float LinePull => RopeTension;

    /// <summary>Maximum pull this winch could produce right now (N), at the
    /// current layer, temperature and battery state.</summary>
    public float AvailableLinePull => drum != null && motor != null
        ? drum.LinePull(motor.StallDrumTorque) : 0f;

    /// <summary>Rope speed at the drum surface (m/s). Positive = spooling in.</summary>
    public float LineSpeed => drum != null ? drum.LineSpeed(DrumOmega) : 0f;

    /// <summary>Mechanical power currently going into the rope (W).</summary>
    public float RopePower => Mathf.Abs(RopeTension * LineSpeed);

    public float MotorCurrent => motor != null ? motor.Current : 0f;
    public float BatteryVoltage => motor != null ? motor.TerminalVoltage : 0f;
    public float MotorTemperature => motor != null ? motor.Temperature : 0f;
    public float BrakeTemperature => brake != null ? brake.Temperature : 0f;

    /// <summary>True when the motor is drawing current but the drum is not turning.</summary>
    public bool IsStalled => Mathf.Abs(_duty) > 0.01f &&
                             Mathf.Abs(DrumOmega) < 0.05f &&
                             MotorCurrent > 100f;

    // ---- Internals ----
    private InputAction _moveAction;
    private InputAction _freespoolAction;
    private float _duty;
    private float _motorTorque;
    private float _brakeTorque;
    private float _loadTorque;
    private float _frictionTorque;
    private float _inertia;
    private bool _warnedOverspeed;

    void Awake()
    {
        if (drum == null) drum = GetComponent<WinchDrum>();
        if (motor == null) motor = GetComponent<WinchMotor>();
        if (brake == null) brake = GetComponent<WinchBrake>();
        SetupActions();
    }

    void Start()
    {
        // CableManager spawns the rope in Awake, so it exists by now.
        if (rope == null)
            rope = FindFirstObjectByType<CableSimulator>();

        if (tether == null) tether = GetComponent<WinchRopeTether>();
        if (tether != null) tether.drivenExternally = true;

        CheckConfiguration();

        if (drum != null)
        {
            // The drum starts out with exactly as much rope deployed as the sim
            // currently has free, so the two never disagree on frame one.
            //
            // With no rope chain the drum is seeded from the endpoints instead.
            // It used to be seeded from nothing: DeployedLength stayed 0, so the
            // tether ran at minimumFreeLength against a load metres away, which
            // is a several-metre extension on a rope spring -- an instant
            // clamped-to-MBS yank that threw the load across the scene on frame
            // one. A missing visual rope must not change the load path.
            float deployed = rope != null
                ? rope.cableLength - minimumFreeLength
                : MeasuredSpan() - minimumFreeLength;

            drum.SetDeployedLength(Mathf.Max(0f, deployed));

            float freeLength = drum.DeployedLength + minimumFreeLength;
            if (rope != null) rope.SetLength(freeLength);
            if (tether != null) tether.SetFreeLength(freeLength);
        }
    }

    /// <summary>Straight-line distance the rope has to cover right now (m).</summary>
    float MeasuredSpan()
    {
        if (tether != null && tether.ropeExit != null && tether.loadAttachment != null)
            return Vector3.Distance(tether.ropeExit.position,
                                    tether.loadAttachment.position);
        return minimumFreeLength;
    }

    /// <summary>
    /// Sanity-check the drivetrain against itself at startup. These are the
    /// mistakes that produce a sim that runs happily and is simply wrong, which
    /// are the expensive kind.
    /// </summary>
    void CheckConfiguration()
    {
        if (motor == null || drum == null) return;

        float stall = motor.StallDrumTorque;

        // A winch that can pull harder than its brake can hold will lower the
        // load the instant you let go of the button. Real ones cannot do this.
        if (brake != null)
        {
            float coulomb = motor.CoulombFrictionAt(stall);

            if (brake.holdingTorque + coulomb < stall)
                Debug.LogWarning(
                    $"[Winch] Brake holds {brake.holdingTorque + coulomb:N0} N*m but " +
                    $"the motor can produce {stall:N0} N*m. The winch can pull loads " +
                    $"it cannot hold, so they will run back out as soon as power is " +
                    $"cut. Raise WinchBrake.holdingTorque to at least {stall:N0}.", this);

            // Powering out is the harder case: the motor adds to the load
            // instead of opposing it, and the brake is at its reduced
            // power-out engagement while it does.
            float payoutHold = brake.holdingTorque * brake.powerOutEngagement + coulomb;
            if (payoutHold < stall)
                Debug.LogWarning(
                    $"[Winch] Brake holds {payoutHold:N0} N*m while powering out but " +
                    $"a rated load needs about {stall:N0}. Loads will run away on the " +
                    $"way down even with the brake cold. Raise " +
                    $"WinchBrake.holdingTorque to about " +
                    $"{(stall - coulomb) / Mathf.Max(0.01f, brake.powerOutEngagement):N0}.",
                    this);
        }

        // Backward Euler is stable at any timestep, but the rope tether is still
        // an explicit spring, so the physics rate still matters for that.
        if (Time.fixedDeltaTime > 0.011f)
            Debug.LogWarning(
                $"[Winch] Fixed timestep is {Time.fixedDeltaTime * 1000f:N1} ms. " +
                $"The drivetrain solves fine at any rate, but the rope chain and " +
                $"tether want 5 ms. Expect a soft, laggy line above ~10 ms.", this);

        if (drum.ViolatesMinBendRadius)
            Debug.LogWarning(
                $"[Winch] Drum barrel radius {drum.barrelRadius * 1000f:N1} mm is " +
                $"below the rope's minimum bend radius " +
                $"{drum.rope.MinBendRadius * 1000f:N1} mm. A real rope would be " +
                $"damaged by this drum.", this);
    }

    void SetupActions()
    {
        if (inputActions == null) return;

        var map = inputActions.FindActionMap(actionMapName, throwIfNotFound: false);
        if (map == null)
        {
            Debug.LogWarning($"[Winch] Action map '{actionMapName}' not found in " +
                             $"{inputActions.name}; using keyboard fallback.", this);
            return;
        }

        _moveAction = map.FindAction(moveActionName, throwIfNotFound: false);
        _freespoolAction = map.FindAction(freespoolActionName, throwIfNotFound: false);

        map.Enable();

        if (_moveAction == null)
            Debug.LogWarning($"[Winch] Action '{moveActionName}' not found in map " +
                             $"'{actionMapName}'; using keyboard fallback.", this);
    }

    void FixedUpdate()
    {
        if (drum == null) return;

        float dt = Time.fixedDeltaTime;

        ReadInput();
        if (tether != null) tether.Solve(dt);   // ordered, not left to Unity
        ReadRopeTension();
        SolveDrum(dt);
        ApplySpooling(dt);
        UpdateVisual();
        UpdateState();
    }

    void ReadInput()
    {
        if (!externalControl)
        {
            float raw = 0f;

            if (_moveAction != null) raw = _moveAction.ReadValue<float>();
            else if (useKeyboardFallback && Keyboard.current != null)
            {
                if (Keyboard.current.upArrowKey.isPressed) raw += 1f;
                if (Keyboard.current.downArrowKey.isPressed) raw -= 1f;
            }

            bool wantFreespool = false;
            if (_freespoolAction != null) wantFreespool = _freespoolAction.IsPressed();
            else if (useKeyboardFallback && Keyboard.current != null)
                wantFreespool = Keyboard.current.leftShiftKey.isPressed;

            ClutchEngaged = !(allowFreespool && wantFreespool);

            // A contactor winch is on or off, never partly on.
            _duty = Mathf.Abs(raw) < 0.35f ? 0f : Mathf.Sign(raw);
        }

        if (!ClutchEngaged) _duty = 0f;

        // End stops.
        if (stopAtFullRetract && _duty > 0f && drum.FullySpooled) _duty = 0f;
        if (enforceMinimumWraps && _duty < 0f && drum.AtMinimumWraps) _duty = 0f;
    }

    void ReadRopeTension()
    {
        if (tether != null)
        {
            RopeTension = tether.Tension;
            // Feed it back so the rope's own readouts and failure check agree
            // with the load path that is actually being solved.
            if (rope != null) rope.ReportExternalTension(RopeTension);
        }
        else
        {
            RopeTension = rope != null ? rope.TensionAtStart : 0f;
        }
    }

    /// <summary>
    /// Integrate the drum for one step, fully implicitly.
    ///
    /// The step solves for the drum speed w1 that satisfies
    ///
    ///     I (w1 - w0) / dt  =  T_motor(w1) + T_load - b*w1 + T_friction(w1)
    ///
    /// where every speed-dependent term is evaluated at the END of the step, on
    /// the real torque curve rather than a linearisation of it, and dry friction
    /// is solved as a constraint rather than a torque.
    ///
    /// WHY IT HAS TO BE DONE THIS WAY
    /// Two things here are far stiffer than they look.
    ///
    /// The motor: a series motor sheds its entire stall torque across its
    /// no-load speed, and that slope arrives at the drum multiplied by
    /// gearRatio^2 -- roughly -370 N*m per rad/s against only ~1.5 kg*m^2 of
    /// reflected inertia. Explicit integration is stable only while
    /// dt*|slope|/I &lt; 2 and this project sits at 1.33 even at a 5 ms step. Worse,
    /// below about 1.3 rad/s the motor is hard on its current limit, so the
    /// local slope there is exactly ZERO while the true torque a fraction of a
    /// rad/s later has already collapsed. That is why linearising is not enough
    /// and this solves the actual curve: from rest under 4,000 lb the explicit
    /// step launched the drum to 3.4 rad/s when the steady answer was 1.5, the
    /// motor then made no torque at all, the drum fell back, and it hunted like
    /// that forever. Line speed and current visibly buzzed.
    ///
    /// The friction: 135 N*m of drivetrain Coulomb against 1.5 kg*m^2 is a
    /// 0.45 rad/s velocity change per 5 ms step, nearly ten times the 0.05 rad/s
    /// band the old sign smoothing used to decide direction. So sign(w) flipped
    /// every single step and a stationary drum holding a load buzzed rather than
    /// sat still. Dry friction has no defined value at zero speed -- it is
    /// whatever it needs to be, up to a limit -- and no torque you integrate can
    /// express that. It has to be a constraint.
    ///
    /// Both are exact at steady state, so every fitted spec-sheet number is
    /// unchanged. The result is stable at any timestep, gear ratio or capacity.
    /// </summary>
    void SolveDrum(float dt)
    {
        float inertia = drum.TotalInertia;
        if (ClutchEngaged && motor != null) inertia += motor.ReflectedInertia;
        inertia = Mathf.Max(1e-4f, inertia);
        _inertia = inertia;

        // Rope always pulls in the pay-out direction.
        _loadTorque = -drum.TorqueFromTension(RopeTension);

        if (ClutchEngaged)
        {
            // Powering IN is not a brake release. The brake is one-way, so the
            // winch just overruns it; only powering OUT moves the actuator.
            if (brake != null) brake.UpdateActuation(_duty < -0.01f, false, dt);

            float viscous = motor != null ? motor.ViscousCoefficient : 0f;
            DrumOmega = SolveImplicitStep(DrumOmega, inertia, viscous, dt);

            // With the clutch in, the drum cannot turn faster than the armature
            // it is geared to. Nothing else bounds an overhauling load once the
            // brake has faded, and the unbounded answer is an armature at 65,000
            // rpm -- a state the hardware disassembles rather than reaches.
            if (motor != null)
            {
                float cap = motor.MaxDrumSpeed;
                Overspeed = Mathf.Abs(DrumOmega) > cap;
                if (Overspeed)
                {
                    DrumOmega = Mathf.Clamp(DrumOmega, -cap, cap);
                    if (!_warnedOverspeed)
                    {
                        _warnedOverspeed = true;
                        Debug.LogWarning(
                            "[Winch] Drum has overhauled the drivetrain past the " +
                            "motor's speed limit and is clamped. The load is running " +
                            "away: the brake has faded or is undersized. Readings " +
                            "past this point are outside the model's valid range.", this);
                    }
                }
            }

            // Commit the motor's electrical and thermal state at the speed it
            // actually ended the step at, not the one it started at.
            _motorTorque = motor != null ? motor.Step(_duty, DrumOmega, dt) : 0f;
        }
        else
        {
            // Freespool: the gearbox and brake are mechanically out of the loop,
            // so neither the reflected armature nor the drivetrain friction is
            // there. Only the drum's own bearing drag remains, and that is
            // viscous, so it solves in closed form.
            if (motor != null) motor.Step(0f, 0f, dt);
            if (brake != null)
            {
                brake.UpdateActuation(false, true, dt);
                brake.ReportApplied(0f, DrumOmega, false, dt);
            }

            _motorTorque = 0f;
            _brakeTorque = 0f;
            _frictionTorque = -freespoolDrag * DrumOmega;
            Overspeed = false;   // the drum is off the gearbox; it may spin freely

            float a = inertia / dt + Mathf.Max(0f, freespoolDrag);
            DrumOmega = (inertia / dt * DrumOmega + _loadTorque) / Mathf.Max(1e-6f, a);
        }

        // Kill numerical drift at rest so a held load does not slowly walk.
        if (Mathf.Abs(DrumOmega) < 1e-4f) DrumOmega = 0f;

        DrumAngle += DrumOmega * dt;
    }

    /// <summary>
    /// Solve the implicit step for the end-of-step drum speed.
    ///
    /// The residual
    ///
    ///     g(w) = I (w - w0)/dt + b*w + R(w) - T_motor(w) - T_load
    ///
    /// is monotonically non-decreasing in w: the inertia and viscous terms rise,
    /// motor torque only falls with speed so -T_motor rises, and the resistance
    /// R(w) steps up once as it crosses the speed the friction is holding at. A
    /// monotone residual means bisection cannot miss the root and cannot
    /// diverge, which is exactly the guarantee wanted here.
    ///
    /// That step is what makes friction a real constraint. If g changes sign
    /// across the hold speed, the root IS the hold speed and friction absorbs
    /// precisely what was there and no more. Stiction falls out of the solve
    /// instead of needing a special case.
    /// </summary>
    float SolveImplicitStep(float omega0, float inertia, float viscous, float dt)
    {
        float ia = inertia / dt;
        float coulomb = motor != null ? motor.CoulombFrictionAt(_loadTorque) : 0f;
        float target = BrakeHoldSpeed();

        // Put the drum at the hold speed and ask what resistance it would take
        // to keep it there. Setting w1 = target in the balance and rearranging
        // gives R = -needed, with needed as below. The drum departs the target
        // in the direction of needed, so that is the direction resistance fights.
        float torqueAtTarget = MotorTorqueAt(target) + _loadTorque;
        float needed = ia * (omega0 - target) + torqueAtTarget - viscous * target;
        float dir = needed >= 0f ? 1f : -1f;

        // Static capacity: nothing is sliding yet.
        float brakeStatic = BrakeCapacity(dir, true);

        if (Mathf.Abs(needed) <= coulomb + brakeStatic)
        {
            ReportResistance(-needed, coulomb, brakeStatic, false, dt, target);
            return target;
        }

        // It breaks loose, and slides at the dynamic level.
        float brakeDyn = BrakeCapacity(dir, false);
        float resist = -dir * (coulomb + brakeDyn);

        // Bracket the root. The hold speed is one end -- the residual is known
        // to have the right sign there, because the hold test just failed. The
        // other end is any bound the drum cannot reach in one step; the whole
        // net torque impulse is a hard over-estimate, which is all a bracket has
        // to be. Because the residual is monotone, bisection from a valid
        // bracket cannot miss and cannot diverge.
        float span = Mathf.Abs(omega0 - target) +
                     (Mathf.Abs(torqueAtTarget) + coulomb + brakeDyn) / Mathf.Max(1e-4f, ia) +
                     1f;

        float lo = dir > 0f ? target : target - span;   // residual negative here
        float hi = dir > 0f ? target + span : target;   // residual positive here

        for (int k = 0; k < 40; k++)
        {
            float mid = (lo + hi) * 0.5f;
            float g = ia * (mid - omega0) + viscous * mid - resist
                    - MotorTorqueAt(mid) - _loadTorque;
            if (g < 0f) lo = mid; else hi = mid;
        }

        float omega = (lo + hi) * 0.5f;
        ReportResistance(resist, coulomb, brakeDyn, true, dt, omega);
        return omega;
    }

    /// <summary>
    /// Drum speed the brake is trying to hold this step (rad/s).
    ///
    /// Idle it is zero: the brake is a parking brake and the load must not move.
    ///
    /// Powering out it is the motor's own free-running speed, negative. This is
    /// the part that is easy to get wrong. A winch brake is not released when
    /// you power a load down -- the ramp inside it is unwound by the input shaft
    /// and wound straight back up by the load, so the two balance and the load
    /// descends at whatever speed the motor happens to be turning. Without this,
    /// nothing bounds payout at all: the rope torque simply exceeds drivetrain
    /// friction and the drum accelerates until viscous drag catches it, which on
    /// this winch is 600 fpm under 2,000 lb and 4,200 fpm under 9,500 lb. Both
    /// are nonsense, and both were what the sim did.
    ///
    /// With it, power-out speed is set by the motor and barely moves with load,
    /// which is what the real machine does -- and the brake absorbs the whole
    /// difference as heat, which is why powering a heavy load down is the
    /// documented way to cook a winch brake and why fade matters here at all.
    /// </summary>
    float BrakeHoldSpeed()
    {
        if (_duty >= -0.01f || motor == null) return 0f;
        return -motor.NoLoadDrumSpeed;
    }

    /// <summary>Drum torque the motor would make at a trial speed, no state touched.</summary>
    float MotorTorqueAt(float omega) =>
        motor != null ? motor.DrumTorqueAt(_duty, omega) : 0f;

    /// <summary>
    /// Brake capacity acting against motion in direction <paramref name="dir"/>.
    /// The drum brake is one-way: it stops the load running out and is simply
    /// overrun when the winch pulls in.
    /// </summary>
    float BrakeCapacity(float dir, bool statically)
    {
        if (brake == null) return 0f;
        if (brake.oneWayHoldsPayoutOnly && dir >= 0f) return 0f;
        return statically ? brake.StaticCapacity : brake.DynamicCapacity;
    }

    void ReportResistance(float applied, float coulomb, float brakeCap,
                          bool slipping, float dt, float slipOmega = 0f)
    {
        _frictionTorque = applied;

        // Split it out for telemetry in proportion to what each element
        // contributed to the capacity that produced it.
        float total = Mathf.Max(1e-6f, coulomb + brakeCap);
        _brakeTorque = applied * (brakeCap / total);

        if (brake != null)
            brake.ReportApplied(_brakeTorque, slipOmega, slipping && brakeCap > 0f, dt);
    }

    /// <summary>Power the brake is currently turning into heat (W).</summary>
    public float BrakePower => brake != null
        ? Mathf.Abs(brake.AppliedTorque * DrumOmega) : 0f;

    void ApplySpooling(float dt)
    {
        float deltaAngle = DrumOmega * dt;

        // Sample the radius BEFORE rotating. Rotate changes the spooled length
        // and therefore the radius, so reading it afterwards compared the rope
        // that moved against a radius that no longer applied to it.
        float radiusBefore = drum.EffectiveRadius;
        float requested = deltaAngle * radiusBefore;

        float moved = drum.Rotate(deltaAngle);

        // If the drum hit a hard stop, it cannot keep turning.
        if (Mathf.Abs(requested) > 1e-6f && Mathf.Abs(moved) < Mathf.Abs(requested) * 0.5f)
            DrumOmega = 0f;

        float freeLength = drum.DeployedLength + minimumFreeLength;
        if (rope != null) rope.SetLength(freeLength);
        if (tether != null) tether.SetFreeLength(freeLength);
    }

    void UpdateVisual()
    {
        if (drumVisual == null) return;
        Vector3 axis = drumVisual.parent != null
            ? drumVisual.parent.TransformDirection(drumSpinAxis)
            : drumSpinAxis;
        drumVisual.Rotate(axis, DrumOmega * Mathf.Rad2Deg * Time.fixedDeltaTime, Space.World);
    }

    void UpdateState()
    {
        if (!ClutchEngaged) { State = WinchState.Freespool; return; }
        if (motor != null && motor.ThermalCutout) { State = WinchState.ThermalCutout; return; }
        if (IsStalled) { State = WinchState.Stalled; return; }
        if (_duty > 0f) { State = WinchState.PowerIn; return; }
        if (_duty < 0f) { State = WinchState.PowerOut; return; }
        State = WinchState.Idle;
    }

    // ---- API for UI / external control ----

    /// <summary>
    /// Drive the winch from code instead of input. -1 out, 0 off, 1 in.
    /// Automatically switches the controller into external-control mode so the
    /// input poll cannot race the command back to zero.
    /// </summary>
    public void SetCommand(float duty)
    {
        externalControl = true;
        _duty = Mathf.Clamp(duty, -1f, 1f);
    }

    public void SetFreespool(bool freespool) => ClutchEngaged = !freespool;

    public float MotorTorque => _motorTorque;
    public float BrakeTorque => _brakeTorque;
    public float LoadTorque => _loadTorque;

    /// <summary>Total dry resistance the solver applied last step (N*m), signed.
    /// Includes drivetrain Coulomb friction and the brake.</summary>
    public float FrictionTorque => _frictionTorque;

    /// <summary>Drum-side inertia used by the solver last step (kg*m^2).</summary>
    public float SolverInertia => _inertia;

    /// <summary>Line pull the brake alone can hold at the current layer (N).</summary>
    public float BrakeHoldableLinePull => brake != null && drum != null
        ? brake.HoldableLinePull(drum.EffectiveRadius) : 0f;

    /// <summary>Line pull in pounds-force, the unit every winch is sold in.</summary>
    public float LinePullLbf => LinePull * 0.2248089f;

    /// <summary>Line speed in feet per minute.</summary>
    public float LineSpeedFpm => LineSpeed * 196.8504f;

    void OnValidate()
    {
        minimumFreeLength = Mathf.Max(0.05f, minimumFreeLength);
        if (drumSpinAxis.sqrMagnitude < 1e-6f) drumSpinAxis = Vector3.forward;
    }
}
