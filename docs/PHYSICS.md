# Physics Behind the Simulator

This document explains the physical models and numerical methods used in the 6DOF Rocket Simulator.

## Coordinate Systems

- **World frame**: Earth-fixed, with Z-axis pointing upward
- **Body frame**: Fixed to the rocket, with Z-axis along the longitudinal axis (nose to tail)

Orientation is represented using **quaternions** to avoid gimbal lock.

## Equations of Motion

The 6DOF dynamics are governed by:

### Translational motion
`F = m * a` (in world frame)

Forces considered:
- **Thrust** – acts along body Z-axis, varies with time (from motor thrust curve)
- **Gravity** – constant 9.81 m/s² downward, multiplied by current mass
- **Drag** – opposes velocity: `0.5 * ρ * v² * Cd * A_ref`
- **Normal force (lift)** – simplified model: `0.5 * ρ * v² * CNα * sin(2α) * A_ref`, acting perpendicular to velocity

### Rotational motion
`M = I * α + ω × (I * ω)` (in body frame)

Moments come from:
- **Aerodynamic stability** – normal force acting at CP (center of pressure), creating a restoring moment if CP is behind CG

## Numerical Integration

A 4th‑order Runge‑Kutta (RK4) method integrates the state derivatives over time.

## Motor Model

The Estes A8‑3 thrust curve is approximated by linear interpolation between key points (ignition, peak, tail‑off, burnout). Mass decreases linearly with time during the burn.

## Atmosphere

Density is modelled as an exponential function of altitude: `ρ = ρ₀ * exp(-altitude / H_scale)`

## Limitations

- Simplified normal force model (not full Barrowman equations)
- No wind or atmospheric turbulence
- Constant drag coefficient
- No parachute recovery model
