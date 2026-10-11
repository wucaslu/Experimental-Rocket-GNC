# README
MATLAB/Simulink-based GNC framework for experimental rocketry, with RocketPy used as a reference simulator for trajectory benchmarking, state estimation validation, aerodynamic/environmental modeling comparison, and control system analysis.

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
