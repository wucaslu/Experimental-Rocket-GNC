# README

MATLAB/Simulink-based GNC framework for experimental rocketry, with RocketPy used as a reference simulator for trajectory benchmarking, state estimation validation, aerodynamic/environmental modeling comparison, and control system analysis.

## Current rocket example

The default plant uses the Calisto rocket and Cesaroni M1670 motor parameters from RocketPy's `getting_started.ipynb`: 14.426 kg airframe, 1.815 kg motor hardware, approximately 2.956 kg propellant, and a 3.9 s burn. See the [parameter mapping, numerical assets and assumptions](Navigation/settings/rocketpy_calisto/README.md).

The current integrated benchmark defaults to `RocketScenario='rocketpy-airbrakes'`: the [RocketPy airbrake example](https://docs.rocketpy.org/en/latest/user/airbrakes.html), with standard atmosphere, 1400 m MSL elevation and 0.5° fin cant. `Airbrake.ControlLawMode=1` runs its intended velocity law at 10 Hz using Navigation feedback and onboard IMU burnout inference. Deployment is continuous, changes by at most 0.02 per controller update at or above 1500 m AGL, and holds between updates. The separate root Actuator passes this command directly to physical deployment in this mode. There is no 200 m/s gate, 0.1 command quantization or downward-speed stop; a scheduled update below 1500 m AGL sets deployment to zero. The versioned deployment/Mach Cd data add incremental drag to the nominal powered/coast curves. The example law's approximately 3 km apogee is an outcome; it does not use the target-apogee input.

`Airbrake.ControlLawMode=0` selects the retained target-based apogee predictor; mode 2 retains the earlier constrained RocketPy implementation with its speed gate, quantized commands, descent stop and separate actuator rate limit. These controller selectors are independent of `Rocket.MomentumModel`, which selects the plant equations described below. See the [current exact-logic comparison](Navigation/RocketPlant/validation/rocketpy-airbrakes-exact-v2/Analysis.md); the [earlier constrained comparison](Navigation/RocketPlant/validation/rocketpy-airbrakes-v1/Analysis.md) records the historical mode-2 setup. Select `RocketParameterSet='generic'` for the previous demonstration vehicle.

Both Calisto scenarios use launch coordinates **32.990254°, −106.974998°**, heading **0°**, inclination **85°**, and a physical **5.2 m** rail corresponding to **3.8632 m effective travel**. Initial body attitude is converted from ENU to NED. Weather loads archived tables at initialization; runtime blocks make no forecast calls. To restore the earlier uncontrolled notebook comparison, set `RocketScenario='notebook'` and `RocketWeatherSource='rocketpy-notebook'` before initialization. This uses the GFS snapshot for **2026-10-13 12:00 UTC**, elevation **1471.465942 m MSL**, zero fin cant and retracted airbrakes. `RocketWeatherSource='forecast'` selects the configurable forecast loader; gusts are disabled in both reference comparisons.

For the RocketPy comparison, Calisto selects `Rocket.MomentumModel=1`: the native RocketPy **1.13.0** CM/nozzle vector conventions, coupled momentum equations and axial rail balance. `Rocket.IncludeCoriolis=1` includes its free-flight Earth-rotation term, with the coordinate term removed from Environment accelerometer truth. The generic preset retains `MomentumModel=0`, which follows the physical fixed-origin derivation. These formulations differ in some offset-CM conservation limits; native compatibility should not be treated as independent proof of physical accuracy. See the [momentum validation](Navigation/RocketPlant/validation/rocketpy-momentum-v1/Analysis.md).

## RocketPy references and credits

This project credits the [RocketPy Team and contributors](https://github.com/RocketPy-Team/RocketPy#authors-and-contributors) for their open-source rocket trajectory simulator, technical documentation, and published flight-dynamics formulations. RocketPy was originally created by Giovani Ceotto at Projeto Jupiter.

RocketPy's equations and implementation informed the MATLAB/Simulink rocket plant, particularly the six-degree-of-freedom variable-mass dynamics, burn-dependent center of mass and inertia, component aerodynamic forces and centers of pressure, and fin roll moments. Its separation of rocket, motor, environment, and recovery models also informed the plant architecture. The plant runs through native MATLAB/Simulink functions; its project-specific approximations and extensions are documented in the [rocket plant README](Navigation/functions/aux_Rocket/README.md) and [motor model README](Navigation/functions/aux_Motor/README.md).

Technical references:

- [RocketPy source repository](https://github.com/RocketPy-Team/RocketPy) and [official documentation](https://docs.rocketpy.org/en/latest/).
- [Generalized equations of motion](https://docs.rocketpy.org/en/latest/technical/equations_of_motion_v1.html): coupled translation and rotation with changing mass properties.
- [Flight implementation](https://docs.rocketpy.org/en/latest/_modules/rocketpy/simulation/flight.html): fixed-reference dynamics and moving-center treatment.
- [Aerodynamic surface implementation](https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/aero_surface.html): component forces and moments.
- [Fin aerodynamics and roll implementation](https://docs.rocketpy.org/en/latest/_modules/rocketpy/rocket/aero_surface/fins/fins.html): normal-force slopes, compressibility corrections, and roll moments.
- [Rocket configuration guide](https://docs.rocketpy.org/en/latest/user/rocket/rocket_usage.html): motor installation, aerodynamic surfaces, parachutes, and stability quantities.

Recommended scientific reference, as listed in RocketPy's [citation metadata](https://github.com/RocketPy-Team/RocketPy/blob/master/CITATION.cff):

Ceotto, G. H., et al. (2021). [RocketPy: Six Degree-of-Freedom Rocket Trajectory Simulator](https://doi.org/10.1061/(ASCE)AS.1943-5525.0001331). *Journal of Aerospace Engineering*, 34(6).

RocketPy is distributed under the [MIT License](https://github.com/RocketPy-Team/RocketPy/blob/master/LICENSE), with the upstream copyright notice naming Giovani Hidalgo Ceotto (2018). Credit for the referenced RocketPy work belongs to its authors and contributors.
