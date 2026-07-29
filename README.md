# Realistic Winch Simulator

**Status:** In development — full drivetrain simulated

A Unity winch simulator aiming for the highest achievable accuracy to real-world
winch behaviour. The drivetrain is modelled from physics and fitted to published
manufacturer data rather than tuned by feel.

## Scenes

- **Winch Sim** — the full simulator: motor, gearbox, drum, brake, rope, load.
- **Cable Sim** — the standalone rope playground.

Controls: `↑`/`↓` spool in/out, `Shift` freespool, `F1` toggle telemetry.

## Reference winch

Defaults model a **Warn 9.5xp class 12 V electric recovery winch**:
3/8" × 100 ft 6×25 IWRC wire rope, 156:1 three-stage planetary, 2.5" × 8.75" drum.

### Validation against the published spec sheet

Layer 1 performance, simulated vs. published:

| Line pull | sim speed | published | sim current | published |
|-----------|-----------|-----------|-------------|-----------|
| 0 lb      | 35.7 fpm  | 35.7 fpm  |  70 A       |  70 A     |
| 2,000 lb  | 22.1 fpm  | 26.0 fpm  | 147 A       | 145 A     |
| 4,000 lb  | 17.6 fpm  | 18.7 fpm  | 215 A       | 210 A     |
| 6,000 lb  | 14.9 fpm  | 14.8 fpm  | 280 A       | 285 A     |
| 8,000 lb  | 12.9 fpm  | 12.5 fpm  | 342 A       | 380 A     |
| 9,500 lb  | 11.6 fpm  | 11.4 fpm  | 388 A       | 435 A     |

Maximum pull per drum layer tracks the published table at a constant
1.11–1.13 stall-over-rated margin — the layer falloff curve, which is the single
most characteristic winch behaviour, is reproduced to within 2%.

## What is actually simulated

**Series-wound DC motor** (`WinchMotor`) — saturating field flux
`φ(I) = kφ·I / (1 + I/Isat)`, back-EMF, closed-form current solve, battery sag
under load, resistive heating, thermal derate and cutout. A permanent-magnet
model cannot fit a real winch: the constant that reproduces 9,500 lb of pull
predicts half the real free speed. Series behaviour is why a winch has enormous
stall torque yet still runs fast unloaded.

**Drum with layer buildup** (`WinchDrum`) — wraps per layer, layer radius growth,
smooth effective-radius interpolation, rope-mass inertia, minimum-wrap reserve.
Line pull ∝ 1/radius and line speed ∝ radius, so pull falls by half between the
first and last layer exactly as the spec sheet says.

**Automatic load-holding brake** (`WinchBrake`) — normally engaged, releases only
under power, holds only against payout, finite holding torque with creep once
exceeded, friction heating and fade.

**Drivetrain integrator** (`WinchController`) — solves
`I(t)·dω/dt = T_motor + T_brake + T_friction − T_rope`, where inertia includes the
drum, the spooled rope and the gear-reflected armature. Nothing commands a speed;
speed is whatever the torque balance produces. Freespool disconnects the drum
from the gearbox entirely.

The step is solved **implicitly**, on the real torque curve rather than a
linearisation of it, and dry friction is solved as a **velocity constraint**
rather than a torque. Both are necessary, not stylistic — see below.

**Wire rope** (`WireRopeSpec`, `CableSimulator`, `WinchRopeTether`) — real
diameter, linear density, breaking strength, effective modulus and metallic area.
Under 9 kN a 3/8" rope stretches 0.33%, which is what `stress/E` says it should.

## Rope architecture

The rope is deliberately split in two:

- `WinchRopeTether` solves the **load path** analytically as a single elastic
  element using the rope's real EA.
- `CableSimulator` simulates the **shape** — catenary sag, swing, collision.

This split exists because it was measured to be necessary. A rigidbody chain of
~100 g links pulling a 1,200 kg load is a 12,000:1 mass ratio across a long
series of hard constraints. PhysX cannot solve it: soften the constraints and the
rope silently stretches to twice its length while the drum reads almost no
tension; stiffen them with projection and mass scaling and the solver injects
energy until the load departs at 180 m/s. Both failures were reproduced on this
scene before the split. Solving the load path as the one-dimensional spring it
physically is gives an exact, unconditionally stable result — and the chain is
then free to do the job it is good at.

## Why the drivetrain is solved implicitly

The drivetrain is far stiffer than it looks, in two separate ways, and an
explicit step got both wrong.

**The motor.** A series motor sheds its entire stall torque across its no-load
speed, and that slope arrives at the drum multiplied by `gearRatio²` — roughly
−370 N·m per rad/s against only ~1.5 kg·m² of reflected inertia. Explicit
integration is stable while `dt·|slope|/I < 2`; this project sits at **1.33 even
at a 5 ms step**. Worse, below about 1.3 rad/s the motor is hard against its
current limit, so the local slope there is exactly zero while the true torque a
fraction of a rad/s later has already collapsed — which is why linearising is not
enough and the solver has to evaluate the actual curve.

The measured effect: from rest under 4,000 lb the explicit step launched the drum
to 3.4 rad/s when the steady answer was 1.5, the motor then made no torque at
all, the drum fell back, and it hunted like that forever. Reported line speed
oscillated ±19 fpm around a mean of 13.6 when the true value is 18.8.

**The friction.** 135 N·m of drivetrain Coulomb against 1.5 kg·m² is a 0.45 rad/s
velocity change per 5 ms step — nearly ten times the 0.05 rad/s band the old sign
smoothing used to pick a direction. So `sign(ω)` flipped every single step. A
stationary drum holding a load buzzed rather than sat still. Dry friction has no
defined value at zero speed; it is whatever it needs to be, up to a limit, and no
torque term can express that.

The residual is monotone in ω, so bisection cannot miss the root or diverge, and
stiction falls out of the same solve as a sign change at the hold speed. Both
fixes are exact at steady state, so every fitted number above is unchanged.

Measured, 4,000 lb, powering in:

| timestep | explicit | implicit |
|----------|----------|----------|
| 2 ms  | —              | 18.8 fpm, ripple 0.000 |
| 5 ms  | 13.6 fpm, ripple 19.3 | 18.8 fpm, ripple 0.000 |
| 20 ms | 5.1 fpm, ripple 176.3 | 18.8 fpm, ripple 0.000 |
| 50 ms | −21.3 fpm, ripple 490.6 | 18.8 fpm, ripple 0.000 |

## The brake is not released when you power out

This is the piece that is easiest to get wrong, and getting it wrong is loud.

A winch brake is one-way and load-reactive. Powering **in** does not release it —
the winch simply overruns it. Powering **out** does not release it either: the
ramp inside is unwound by the input shaft and wound straight back up by the load,
so the two balance and the load descends at whatever speed the motor happens to
be turning.

Without that, nothing bounds payout at all. Rope torque exceeds drivetrain
friction and the drum accelerates until viscous drag catches it — 608 fpm under
2,000 lb and 4,235 fpm under 9,500 lb. Both nonsense. And with the brake treated
as released whenever power was on, a load heavier than the motor could pull would
run straight back out while the operator held the IN button.

With it, power-out speed is set by the motor and barely moves with load, which is
what the real machine does, and the brake absorbs the entire difference as heat:

| line pull | payout speed | brake after 15 s |
|-----------|--------------|------------------|
| 0 lb      | 39.8 fpm | 30 °C |
| 2,000 lb  | 39.8 fpm | 58 °C |
| 6,000 lb  | 39.8 fpm | 112 °C |
| 9,500 lb  | 39.8 fpm, then runs | 160 °C |

Hold a rated load on the way down and the brake fades out from under it at about
20 s, the drum runs to the drivetrain's speed limit, and it sits there cooking.
That is the documented reason winch manuals warn about long heavy power-outs, and
it is now reachable in the sim rather than merely described in a comment.

Brake heat is no longer gated on a "slipping" flag either — a brake modulating a
load down is holding it and still burning every watt of it, which was exactly the
case the old gate threw away.

## Where the model stops

`WinchMotor.maxArmatureRpm` clamps the drum and raises `WinchController.Overspeed`.
A series motor makes no torque above its free speed and cannot usefully
regenerate, so once the brake has faded out from under a heavy load the only thing
left resisting is viscous drag — and solving that balance honestly puts the drum
at 21 m/s and the armature at 65,000 rpm. No drivetrain reaches that; it comes
apart first. Rather than extrapolate a fitted motor curve into a state the
hardware cannot occupy, the sim clamps and says so.

## Validation

`WinchValidationHarness` drives a scripted duty cycle in play mode and records
telemetry, so the sim can be checked against a spec sheet rather than judged by
eye. Attach it, point it at the winch, call `Begin`, read `Report`.

`WinchController` also sanity-checks itself on startup and warns if the brake
cannot hold what the motor can pull, if the brake cannot control a rated load on
the way down, or if the drum bends the rope tighter than its minimum bend radius.

## Physics settings this project needs

- Fixed timestep **0.005 s** — the rope chain is stiff. The drivetrain no longer
  cares: it gives the same answer from 2 ms to 100 ms.
- Solver iterations **24 / 12** project-wide, **40 / 20** per rope node.
- Script execution order does **not** matter for the winch. `WinchController`
  drives `WinchRopeTether.Solve` itself, because when both ran their own
  `FixedUpdate` the order decided whether the drum loaded against this step's
  rope length or last step's, and one frame of lag in a loop this stiff is what
  decides whether it damps or grows.
