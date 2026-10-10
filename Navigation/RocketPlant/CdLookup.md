# CSV-backed drag lookup

The lookup uses CM air-relative speed in m/s and a control level from 0 to 1.
Its source is a CSV with columns `Cd,velocity,level`:

```csv
Cd,velocity,level
0.50,0,0.0
0.55,0,0.1
```

This excerpt shows the format; a usable file must contain a complete grid.
Include every combination of your velocity nodes and the 11 control levels
`0,0.1,...,1`, with at least two distinct nonnegative velocities. Cd must be
finite and nonnegative. Row order does not matter. The importer matches
trimmed column headers without regard to case or column order, and permits
extra columns. It rejects duplicate pairs, missing pairs, invalid numbers
and off-grid levels. Levels within 1e-9 of a grid node are canonicalized.

Select the CSV path in `Navigation/settings/rocketCdLookupSettings.m` using
`CdLookupFile`. The default file, `Navigation/settings/cd_lookup_example.csv`,
contains **demonstration data**: a coast-like zero-control curve referenced
to fixed sound speed 340 m/s, and Cd increasing by 0.05 at each 0.1 control
step. Replace it with measured or CFD coefficients for your device.

`rocketPlantLoadCdLookupCsv` reads and validates the file once during model
initialization. It builds numeric `CdLookup.VelocityBreakpoints`,
`CdLookup.ControlBreakpoints` and `CdLookup.Table`. Runtime interpolation
uses that numeric table, without file reads or persistent variables.

```matlab
CdLookup = rocketPlantLoadCdLookupCsv('your_cd_data.csv');
[Cd,usedQuery] = rocketPlantCdLookup(250,0.5,CdLookup);
```

Interpolation is bilinear in velocity and control. Finite nonnegative speed
and finite control queries outside the axes are clamped to their edges;
negative speed and nonfinite queries are rejected. The 0.1 spacing specifies
table nodes; intermediate commands such as 0.35 are interpolated smoothly. `usedQuery`
contains `[clampedSpeed;clampedControl]`.

## Model inputs and selection

| Root port | Signal | Role |
| --- | --- | --- |
| 1 | `Cd_control_level` | Lookup control, clamped to [0,1] |
| 2 | `Cd_lookup_enabled` | Enable lookup (0/1) |

These scalar-double root inputs feed Environment ports 3 and 4. The retained
manual Constants feed Environment ports 1 and 2: SID `1604` supplies the
absolute `Cd_command` (saved value 1), and SID `1600` supplies
`Cd_override_enabled` (saved value 0). Edit their values in the model or use
`SimulationInput.setBlockParameter` with their resolved paths; see
[CdControl.md](CdControl.md). The unused `Control` placeholder is separate
from the lookup input.

Manual override has priority when both enables are true. Otherwise lookup
enable selects the CSV coefficient; with both enables zero, the original
powered/coast Mach curves remain active. Lookup
is an absolute coefficient and does not automatically switch motor modes.
For the demonstration it is enabled only after burnout.

Environment's `Cd_lookup` subsystem converts NEU truth velocity to NED and
subtracts the shared Weather NED wind before taking its norm. It uses truth
velocity, with no GNSS velocity aiding or sensor changes. The lookup and
selection run at `Rocket.Ts`; the selected Cd is held throughout the plant's
internal RK4 stages, matching the existing external-Cd contract.

The body axial drag and existing canopy fade use the selected coefficient.
Other component and parachute coefficients retain their own calculations.
`out.RocketCdControl` retains its existing five-column format for the selected
command. `out.RocketCdLookup` logs seven columns:

| Column | Meaning |
| --- | --- |
| 1 | Actual CM air-relative speed, m/s |
| 2 | Clamped lookup speed, m/s |
| 3 | Raw control command |
| 4 | Clamped control command |
| 5 | Lookup Cd |
| 6 | Lookup enable (0/1) |
| 7 | Selected mode: 0 nominal, 1 lookup, 2 direct |

Run `Navigation/runRocketCdLookupExample.m` to command level 0.5 from 53 to
65 s. External Dataset elements map by port order; supply the two lookup
elements and keep the manual enable zero. `runRocketCdExample.m` configures
the retained manual Constants for its direct-Cd demonstration. The earlier
timed direct-input validation used the model's former root interface.

## Checks

`checkRocketCdLookup` covers independent bilinear references, clipping,
numeric classes, wind/frame handling, source priority and physical load
propagation. `checkRocketCdLookupCsv` covers shuffled rows, header variants,
schema/grid errors and fresh reloads. The pre-lookup model is preserved under
`validation/cd-lookup-v0-baseline/`; check results are saved under
`validation/cd-lookup-v1/`.

All 14 new lookup/CSV suites and eight existing direct-Cd suites passed.
A 65 s disabled-lookup run reproduced all 17 baseline plant states exactly.
The full 350 s example selected lookup mode exactly from 53 to 65 s; its
effective Cd matched the table output exactly and ranged from 0.7500 to
0.9721. Airspeed agreed with the plant diagnostics within 1.71e-13 m/s.
States through activation matched the baseline exactly. The maximum
quaternion norm error was 2.23e-16. A separate integrated check confirmed
manual priority when both enables were one.

| Demonstration result | Nominal curves | Lookup level 0.5, 53–65 s |
| --- | ---: | ---: |
| Apogee, m AGL | 3156.264 | 2727.956 |
| Landing, s | 227.960 | 210.960 |

Integrated evidence is in `validation/cd-lookup-v1/integrated-cd-lookup-checks.json`
and `half-control-coast/rocket-full-simulation.mat`. The current model omits
the optional navigation position logs; the exporter therefore reports plant
metrics without navigation error metrics. The new lookup subsystem has no
connectivity issues. Two unconnected ports on the existing Control placeholder
and four previous root/magnetic warnings remain. The saved model before
these lookup edits, including your manual Constants and Control placeholder,
is preserved under `validation/cd-lookup-v0-current-model/`.
