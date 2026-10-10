# Manual body drag control

`NavigationRocketPlant.slx` retains two manual Constant blocks, forwarded
through Environment to `Rocket_dynamics` and its step and force adapters:

| Block SID | Signal | Meaning | Saved value |
| --- | --- | --- | --- |
| 1604 | `Cd_command` | Absolute, dimensionless axial body drag coefficient | 1 |
| 1600 | `Cd_override_enabled` | Enable the manual command (0/1) | 0 |

These Constants feed Environment ports 1 and 2. Their outputs are scalar
doubles; the pure MATLAB helpers also accept a logical enable.

The model's two root inputs are now `Cd_control_level` (port 1) and
`Cd_lookup_enabled` (port 2), connected to Environment ports 3 and 4 for the
CSV-backed velocity/control lookup. Manual override has priority over the
lookup. When both enables are zero, the nominal curves apply. See
[CdLookup.md](CdLookup.md). The unused `Control` placeholder remains separate.

The command accepts any finite, real, nonnegative numeric scalar. Enable
accepts a logical scalar or numeric 0/1. Invalid inputs raise
`RocketPlant:InvalidCdControl`, including when the override or aerodynamics
is disabled. Numeric commands are converted to double before use. The saved
manual enable is zero; missing external lookup data produces zero lookup
inputs, preserving the nominal model.

The nominal coefficient still comes from `Rocket.AeroCdPowered` or
`Rocket.AeroCdCoast` at the current CM Mach number. When enabled, the command
replaces that coefficient. Axial force uses the existing forward-flight law:

```matlab
Cd_effective = Cd_nominal;
if Cd_override_enabled
    Cd_effective = Cd_command;
elseif Cd_lookup_enabled
    Cd_effective = Cd_lookup_value;
end
Fz_body = -0.5*rho*speedAirCM^2*Rocket.ReferenceArea*Cd_effective;
```

The command affects axial body drag at the CM. Fin/component normal forces,
moments, CPs and canopy CdS are calculated as before. `AeroEnabled` and the
existing recovery fade still scale body loads; body control has zero force
authority once a canopy has fully inflated. Support/rail constraints apply
as before. Input sampling is `Rocket.Ts` (default 0.02 s). Both command and
enable are held constant across all internal RK4 substeps and stages. There
is no persistent state or actuator dynamics in this interface.

## Configuring the manual Constants

Edit the Constant block values in the model, or resolve their SIDs before
applying temporary simulation parameters. This example selects Cd 0.9 for
the whole run and explicitly disables both lookup inputs:

```matlab
run('Navigation/rocketPlantMain.m');
load_system(fullfile(rocketNavigationDir,'RocketPlant','NavigationRocketPlant.slx'));
manualCdBlock = getfullname(Simulink.ID.getHandle('NavigationRocketPlant:1604'));
manualEnableBlock = getfullname(Simulink.ID.getHandle('NavigationRocketPlant:1600'));
t = [0;350];
inputs = Simulink.SimulationData.Dataset;
inputs = inputs.addElement(timeseries(zeros(2,1),t),'Cd_control_level');
inputs = inputs.addElement(timeseries(zeros(2,1),t),'Cd_lookup_enabled');
in = Simulink.SimulationInput('NavigationRocketPlant');
in = in.setBlockParameter(manualCdBlock,'Value','0.9');
in = in.setBlockParameter(manualEnableBlock,'Value','1');
in = in.setExternalInput(inputs);
out = sim(in);
```

The Dataset elements map by **port order**, not by their labels. Temporary
`SimulationInput` overrides leave the saved Constant values intact.
See the MathWorks [`setBlockParameter` documentation](https://www.mathworks.com/help/simulink/slref/simulink.simulationinput.setblockparameter.html)
for simulation-scoped block parameters.
`Navigation/runRocketCdExample.m` demonstrates this interface. The ordinary
`Navigation/runRocketPlant.m` uses the nominal curves with both enables zero.
The prior 53-to-65 s direct-input scenario and its archived results below
used the former root command/enable interface.

## Pure MATLAB helpers

The final two optional arguments are `cdCommand, cdOverrideEnabled`:

```matlab
[forceB,momentB,aero] = rocketPlantAerodynamics(x,t,Rocket,Weather,gustNED,cdCommand,enabled);
[aN,alphaB,dbg,cdInfo] = rocketPlantLoads(x,t,g,Rocket,Weather,gustNED,cdCommand,enabled);
dx = rocketPlantDerivative(x,t,g,Rocket,Weather,gustNED,cdCommand,enabled);
xNext = rocketPlantStep(x,t,g,Rocket,Weather,gustNED,cdCommand,enabled);
[fB,BB,dbg,cdInfo] = rocketPlantForces(x,t,g,BNED,Rocket,Weather,gustNED,cdCommand,enabled);
```

Omitting the arguments defaults to command 0 and enable false. Older calls
retain their behavior. Aerodynamic diagnostics provide `aero.cdNominal`,
`aero.cdCommand`, `aero.cdOverrideEnabled` and `aero.cd` (effective).
The optional fourth Loads/Forces output is the column vector
`[nominal; command; double(enabled); effective; bodyScale]`.

In Simulink, `out.RocketCdControl` records those five columns as a timeseries.
The existing `RocketPlantDebug` log keeps its original 18-column format.

## Historical direct-input validation

`Navigation/tests/checkRocketCdControl.m` checks baseline compatibility,
powered/coast override selection, independent drag-force and closed-form
trajectory references, coordinate/IMU forwarding, major-step command holding,
recovery behavior, and invalid-input rejection. Integrated results are saved
under `validation/cd-control-v1/`; the pre-edit model and helper source are
preserved under `validation/cd-control-v0-baseline/`.

These integrated results were recorded with the former root `Cd_command`
and `Cd_override_enabled` inputs. They validate the unchanged plant override
contract; timed direct root inputs are absent from the current saved model.

All eight Cd suites and the 28 existing physics/aerodynamics/weather suites
passed. The independent force oracle agreed within 5.69e-14 N; the controlled
coast and constant-thrust trajectory references agreed within 1.43e-13 m and
1.43e-14 m/s. A full 350 s run with the override disabled reproduced every
logged plant state of the previous weather-neutral run exactly (maximum
absolute difference 0).

The external-input demonstration commanded Cd 0.9 from 53 to 65 s. Its log
confirmed exact enable boundaries and coefficient selection. States up to
53 s matched the nominal run exactly. Both 350 s simulations completed with
finite states and navigation outputs, and maximum quaternion norm error
2.23e-16 or less.

| Result | Nominal curves | Coast override |
| --- | ---: | ---: |
| Apogee, m AGL | 3156.264 | 2569.463 |
| Apogee event, s | 73.883 | 70.693 |
| Peak speed, m/s | 329.861 | 329.861 |
| Landing, s | 227.960 | 204.460 |

The apogee change was -586.801 m for these demonstration parameters. Sensors
and Navigation subsystem files, their root connections and all original
Stateflow XML contents were unchanged. The structural check retains the
same four pre-existing magnetic/root warnings, with no new structural issues.
