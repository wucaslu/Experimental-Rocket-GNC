# Rocket dynamics truth source

This directory contains an original, explicit-state six degree-of-freedom
MATLAB implementation patterned after the structure of RocketPy. It does not
require Python, RocketPy, Aerospace Toolbox or a trajectory data file.

Open `../../RocketPlant/NavigationRocketPlant.slx` for the integrated model,
or run `../../runRocketPlant.m` to simulate and export results. Edit
`../../settings/rocketPlantSettings.m` to configure the motor and rocket.
The integrated model still uses the project's existing Environment and GNSS
toolbox functions. The statement about toolbox independence applies to these
pure physics helpers.

`rocketPlantBlock` is a stateless Level-2 MATLAB S-function adapter for host
simulation. It has no internal flight memory: the model's Unit Delay owns
all 17 states. This adapter is not configured for embedded code generation.
The CSV source is retained as a commented subsystem in the new model. The
original saved model and source CSVs are preserved. Runtime initialization
uses `rocketPlantMain.m`, which never reads trajectory files.

`rocketPlantInitialize(Rocket)` creates a 17-element state. `rocketPlantStep`
advances that state by `Rocket.Ts`, using fourth-order Runge–Kutta substeps no
larger than `Rocket.MaxStep`. `rocketPlantKinematics` and `rocketPlantForces`
produce the existing Environment interfaces and physical sensor truth.

`rocketPlantAerodynamics`, `rocketPlantLoads`, `rocketPlantDerivative`,
`rocketPlantStep` and `rocketPlantForces` accept two final optional arguments:
`cdCommand, cdOverrideEnabled`. The absolute dimensionless command must be
finite, real, scalar and nonnegative. Enable is logical or numeric 0/1.
With enable false (the default), the nominal powered/coast Mach curve is used;
with enable true, `cdCommand` replaces body axial Cd. Both inputs are held
during each `rocketPlantStep` call. Normal forces and canopy CdS use their
existing coefficients, including the body-aero fade during canopy inflation.
See `../../RocketPlant/CdControl.md` and `../../runRocketCdExample.m`.
`rocketPlantLoads` and `rocketPlantForces` have an optional fourth output
`[Cd_nominal;Cd_command;enable;Cd_effective;bodyScale]`, logged as
`RocketCdControl` in Simulink. The original `dbg` output remains 18 elements.

State layout:

| Elements | Meaning | Unit |
| --- | --- | --- |
| 1:3 | Local North, East, Down position, origin at launch site | m |
| 4:6 | Local North, East, Down velocity | m/s |
| 7:10 | Active Hamilton body-to-NED quaternion `[x;y;z;w]` | — |
| 11:13 | Body angular velocity | rad/s |
| 14 | Phase: 0 pad, 1 rail, 2 free flight, 3 drogue, 4 main, 5 landed | — |
| 15 | Recorded apogee time; `-1` until detected | s |
| 16 | Scheduled drogue deployment time; `-1` until scheduled | s |
| 17 | Scheduled main deployment time; `-1` until scheduled | s |

Body +Z is longitudinal toward the nose. Inclination is above the horizontal,
heading clockwise from North and roll about body +Z. Position output uses
`[North;East;AltitudeMSL-Down]` and velocity uses `[North;East;-vDown]`.
MSL truth altitude is converted to GNSS ellipsoid height by the existing
receiver model. Specific force is computed as
`R'*(aN-[0;0;g]) + alphaB cross IMUOffset + omegaB cross (omegaB cross IMUOffset)`.
Support acceleration at the pad, rail and landed state is included.

The motor curve is piecewise linear. Propellant consumption is proportional
to its exactly integrated impulse, including partial curve segments. Powered
and coast axial drag are interpolated against CM Mach, with drag mode selected
by the motor burn interval. Body axial drag acts at the CM. `rocketPlantBuildAero`
derives separate nose, fin-set and tail CPs and normal-force coefficients from
geometry. `rocketPlantAerodynamics` evaluates each component's local air
velocity, including angular velocity and wind at the rotated CP height.
The fin set includes compressibility, fin/body and fin-count corrections,
cant roll forcing and roll damping. The tail's signed normal-force slope is
preserved. Local CP velocity produces pitch/yaw damping without a fitted
lumped coefficient. Wind is linearly interpolated against AGL altitude.
Density, pressure and speed of sound use standard-atmosphere layers and
geopotential altitude by default. Optional `Weather` and explicit `gustNED`
arguments select the station-anchored moist profile and synthetic wind
disturbances. `rocketWeatherBuildProfile` constructs hydrostatic tables at
initialization; `rocketWeatherConditions` supplies consistent local pressure,
density and sound speed, and `rocketWeatherWind` evaluates wind at each CP.
`rocketWeatherGust` has no persistent variables or random streams. The live
Environment weather block supplies gust and pressure signals. See
`../../RocketPlant/Weather.md` and `../../settings/weatherSettings.m`.
Drogue deployment follows apogee plus latency; main
deployment follows a descending AGL threshold plus its separate latency.
Both CdS values grow with a finite smoothstep inflation and their forces add.
Deployment changes force without resetting position or velocity.
The body aerodynamic contribution smoothly fades to zero as the first
canopy inflates, while this model keeps its 6DOF attitude/attachment moments.
This is an extension to RocketPy's parachute descent phase switch.

The flight phase and deployment times are explicit states. Crossing times
are localized by linear interpolation within one internal substep. Rail
release is resolved at the substep boundary. Only plane ground impact resets
velocity and angular rate. A smaller `MaxStep` improves event resolution.

The model approximates dry and propellant centers of mass as coincident and
fixed. Inertia is diagonal and declines with remaining propellant. The angular
equation is

```
I .* alphaB = M - cross(omegaB,I.*omegaB)
              + (massRate*NozzleGyration-inertiaRate).*omegaB
```

`NozzleGyration` is an approximate diagonal exhaust-stream tensor in m². This
includes an angular exhaust-flux correction but omits the full moving-CM
coupling in RocketPy. CP, thrust offset, canopy attachments and IMU offset are
all measured from the common CM in body coordinates. Optional residual
angular damping is dimensional N·m·s and defaults to zero. Linear Barrowman
normal force is extrapolated at large angle of attack; axial body drag uses
the forward-flight convention rather than a general stalled/reverse-flow law.
This implementation also omits Earth rotation, flexible
bodies, canopy swing, measured stochastic turbulence and detailed
motor internal ballistics. The demonstration settings require measured motor,
mass, inertia, aerodynamics and recovery inputs before predicting a real flight.

`dbg` is a fixed 18-element vector:

| Index | Meaning |
| --- | --- |
| 1 | Phase |
| 2 | Mass, kg |
| 3 | Thrust, N |
| 4 | Ground speed, m/s |
| 5 | CM air speed, m/s |
| 6 | CM airspeed Mach |
| 7 | Density, kg/m³ |
| 8 | Pressure, Pa |
| 9 | Inflated drogue CdS, m² |
| 10 | Inflated main CdS, m² |
| 11 | AGL altitude, m |
| 12:14 | Apogee, drogue and main times, s |
| 15:17 | NED translational acceleration, m/s² |
| 18 | Angular acceleration norm, rad/s² |

References for the architecture and equations:

- [RocketPy equations of motion](https://docs.rocketpy.org/en/latest/technical/equations_of_motion_v1.html)
- [RocketPy rocket configuration](https://docs.rocketpy.org/en/latest/user/rocket/rocket_usage.html)
- [RocketPy parachute configuration](https://docs.rocketpy.org/en/latest/reference/classes/Parachute.html)
- [RocketPy component aerodynamic forces](https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/aero_surface.html)
- [RocketPy fin aerodynamics and roll](https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/fins/fins.html)
- [NASA standard-atmosphere tables](https://ntrs.nasa.gov/citations/19770009539)
