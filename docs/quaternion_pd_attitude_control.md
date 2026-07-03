# Quaternion PD Attitude-Control Demo

This demo adds the first closed-loop control case to the spacecraft dynamics and control testbed. It simulates a rigid spacecraft rotating from an initial attitude error toward a desired inertial pointing attitude using quaternion kinematics, Euler rotational dynamics, and a proportional-derivative attitude-control law.

## Control objective

Drive the spacecraft from an initial attitude offset and nonzero body rates to the desired quaternion:

```text
q_desired = [1, 0, 0, 0]
omega_desired = [0, 0, 0] rad/s
```

The script currently uses an initial attitude of approximately 50 degrees about the body y-axis and an initial body-rate vector of `[1.5, -2.0, 0.8] deg/s`.

## Dynamics model

The simulation combines quaternion kinematics with Euler's rigid-body rotational equations:

```text
q_dot = 0.5 * q ⊗ [0, omega]
I * omega_dot + omega × (I * omega) = tau
```

The inertia matrix used in the demo is diagonal:

```text
I = diag([12, 10, 8]) kg*m^2
```

## Control law

The controller uses the vector part of the shortest-rotation quaternion error and body angular rate feedback:

```text
tau = -Kp * q_error_vector - Kd * omega_body
```

The current gains are:

```text
Kp = [0.45, 0.45, 0.45]
Kd = [5.0, 5.0, 5.0]
```

The commanded torque is clipped to `±0.25 N*m` per axis to represent a simple actuator limit.

## How to run

From the repository root:

```bash
python scripts/run_quaternion_pd_demo.py
```

The script prints final convergence metrics and saves a figure to:

```text
figures/quaternion_pd_attitude_control.png
```

## Outputs

The generated plot shows:

1. Attitude-error convergence in degrees
2. Body-rate damping in deg/s
3. Commanded control torque in N*m
4. Quaternion component history

This demo is intentionally simple, but it establishes the core closed-loop GNC workflow: define dynamics, compute attitude error, command control torque, propagate the states, and validate convergence using plots and final metrics.

## Future improvements

- Add gain-tuning studies and compare underdamped, overdamped, and saturated cases
- Add reaction-wheel actuator dynamics instead of direct body torque
- Add gyro noise and estimated-rate feedback
- Add disturbance torques such as gravity-gradient torque, drag torque, and solar radiation pressure torque
- Add automated tests for quaternion normalization, control torque direction, and convergence behavior
