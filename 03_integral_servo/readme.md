
# 03_integral_servo — Integral-Type Servo System

## Blog Article

[State Feedback Control and State-Space Design: A Comprehensive Guide](https://blog.control-theory.com/entry/state-feedback-control-eng)
— Section: "Integral-Type Servo System"

## Description

This script demonstrates the integral-type servo system, which augments state feedback with an integrator to achieve zero steady-state tracking error for step reference inputs. The key advantage over a pure regulator with feedforward gain is robustness: the integrator rejects constant disturbances and compensates for model uncertainty.

Three designs are compared:
1. **Pole placement**: Augmented system poles placed at {−1, −2, −3, −1.5}
2. **LQR with moderate integral weight** (Q_I = 10): Balanced response
3. **LQR with strong integral weight** (Q_I = 100): Faster tracking, more aggressive control

A baseline comparison is provided against a regulator with feedforward gain (no integrator), showing that feedforward alone cannot reject step disturbances.

## Generated Figures

| File | Description |
|------|-------------|
| `fig/integral_servo_step_comparison.png` | Step response: regulator + feedforward vs integral servo |
| `fig/integral_servo_design_comparison.png` | Comparison of 3 integral servo designs (pole placement, LQR Q_I=10, LQR Q_I=100) |
| `fig/integral_servo_error_integrator.png` | Tracking error convergence and integrator state x_I(t) |
| `fig/integral_servo_pole_map.png` | Pole map showing all designs |
| `fig/integral_servo_disturbance_rejection.png` | Disturbance rejection: integrator eliminates steady-state error from step disturbance |

## Usage

### Python

```bash
cd 03_integral_servo/python
pip install numpy scipy matplotlib control
python integral_servo.py
```

### MATLAB

```matlab
cd 03_integral_servo/matlab
integral_servo
```

Figures are saved to `03_integral_servo/fig/`.

## Plant Model

Same 3rd-order system used throughout this repository:

```
A = [0 1 0; 0 0 1; 1 -1 -1],  B = [0; 0; 1],  C = [1 0 0]
```

Open-loop eigenvalues: 0.544, −0.772 ± 1.115j (unstable)

## Augmented System

```
A_a = [A, 0; -C, 0],  B_a = [B; 0]
```

Control law: u(t) = −K_x·x(t) − K_I·x_I(t), where ẋ_I = r − y

## Design Parameters

- **Regulator baseline**: K via pole placement at {−1, −2, −3}, feedforward N_bar = 6.0
- **Integral servo Design 1**: Augmented poles at {−1, −2, −3, −1.5}
- **Integral servo Design 2**: LQR with Q = diag(1, 1, 1, 10), R = 1
- **Integral servo Design 3**: LQR with Q = diag(1, 1, 1, 100), R = 1
- **Disturbance scenario**: Step disturbance d = 0.5 applied at t = 5 s
