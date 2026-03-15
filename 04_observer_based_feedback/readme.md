# 04_observer_based_feedback — Separation Principle Demo

## Blog Article

[State Observer for State Space Model](https://blog.control-theory.com/entry/2024/10/01/143305)

Hub section: [Observer-Based Feedback and the Separation Principle](https://blog.control-theory.com/entry/state-feedback-control-eng)

## Description

This script provides a **lightweight demonstration of the separation principle** using the same 3rd-order plant as `01_pole_placement/` through `03_integral_servo/`. The controller gain K is designed via pole placement (same as 01), and the observer gain L is designed independently. The simulation shows that the full closed-loop poles are the union of the controller poles and the observer poles — verifying the separation principle.

This folder does **not** cover detailed observer design (gain tuning, different observer types, noise handling). For those topics, see the Related Repository section below.

## Generated Figures

| File | Description |
|------|-------------|
| `fig/observer_based_feedback_output_comparison.png` | Output and control input: full-state feedback vs observer-based feedback |
| `fig/observer_based_feedback_state_estimation.png` | True state x(t) vs estimated state x̂(t) for all 3 state variables |
| `fig/observer_based_feedback_estimation_error.png` | Estimation error e(t) = x(t) - x̂(t) convergence |
| `fig/observer_based_feedback_pole_map.png` | Pole map showing separation: controller poles, observer poles, and full closed-loop poles |

## Usage

### Python

```bash
cd 04_observer_based_feedback/python
pip install numpy scipy matplotlib control
python observer_based_feedback.py
```

### MATLAB

```matlab
cd 04_observer_based_feedback/matlab
observer_based_feedback
```

Figures are saved to `04_observer_based_feedback/fig/`.

## Plant Model

Same 3rd-order system used throughout this repository:

```
A = [0 1 0; 0 0 1; 1 -1 -1],  B = [0; 0; 1],  C = [1 0 0]
```

Open-loop eigenvalues: 0.544, −0.772 ± 1.115j (unstable)

## Design Parameters

- **Controller poles (A−BK):** {−1, −2, −3} (same as 01_pole_placement)
- **Observer poles (A−LC):** {−5, −6, −7} (placed 2–3× faster than controller poles)
- **Initial state mismatch:** x₀ = [1; 0; 0], x̂₀ = [0; 0; 0]

## Related Repository

This folder provides a minimal demonstration of the separation principle
using the same plant model as `01_pole_placement/` through `03_integral_servo/`.

For comprehensive observer design — including Kalman filter, H∞ filter,
multi-rate observer, and outlier-robust MCV observer — see:
- **MATLAB**: [MATLAB_state_observer](https://github.com/Hiroshi-Okajima/MATLAB_state_observer)
- **Python**: [python_state_observer](https://github.com/Hiroshi-Okajima/python_state_observer)
- **Blog hub**: [State Observer and State Estimation: A Comprehensive Guide](https://blog.control-theory.com/entry/state-observer-estimation)
