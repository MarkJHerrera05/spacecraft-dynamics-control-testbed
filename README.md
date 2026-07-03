# Python-Based Spacecraft Dynamics and Control Testbed

This project is a modular Python simulation environment for spacecraft orbital motion, attitude dynamics, sensors, actuators, and control algorithms.

## Project Objectives

- Simulate two-body orbital motion around Earth
- Add perturbation models such as J2
- Propagate spacecraft attitude using quaternions
- Simulate sensors such as gyroscopes and sun sensors
- Simulate actuators such as reaction wheels and magnetorquers
- Implement attitude control laws
- Visualize orbital and attitude behavior

## Current Version

Version 0.2: Two-body orbit propagation and closed-loop quaternion PD attitude-control demo

## Implemented Capabilities

- Two-body orbital dynamics model
- Quaternion utility functions for normalization, multiplication, conjugation, error calculation, kinematics, and angle extraction
- Euler rigid-body rotational dynamics model
- Quaternion PD attitude controller with proportional attitude-error feedback, derivative body-rate feedback, and optional torque saturation
- Closed-loop attitude-control demo that saves convergence plots for attitude error, body rates, control torque, and quaternion history

## Run the Quaternion PD Attitude-Control Demo

From the repository root:

```bash
python scripts/run_quaternion_pd_demo.py
```

The script prints final convergence metrics and saves a figure to:

```text
figures/quaternion_pd_attitude_control.png
```

The demo is documented in:

```text
docs/quaternion_pd_attitude_control.md
```

## Planned Modules

1. Orbit propagation
2. Attitude propagation
3. Environmental disturbances
4. Sensor models
5. Actuator models
6. Control algorithms
7. Visualization tools
8. Automated simulation scripts

## Repository Structure

```text
spacecraft-dynamics-control-testbed/
├── src/
│   ├── orbit/
│   ├── attitude/
│   │   ├── quaternion.py
│   │   └── rigid_body.py
│   ├── control/
│   │   └── quaternion_pd.py
│   └── sensors/
├── scripts/
│   └── run_quaternion_pd_demo.py
├── figures/
├── docs/
│   └── quaternion_pd_attitude_control.md
├── README.md
├── requirements.txt
└── .gitignore
```
