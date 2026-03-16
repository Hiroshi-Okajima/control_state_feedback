# control_state_feedback

MATLAB and Python simulation code for state feedback control design — pole placement, LQR, integral servo, observer-based feedback, and LMI-based design.

Companion code for [blog.control-theory.com](https://blog.control-theory.com/).

**Author**: [Hiroshi Okajima](https://github.com/Hiroshi-Okajima), Associate Professor, Kumamoto University, Japan  
**Blog**: https://blog.control-theory.com/  
**Hub article**: [State Feedback Control and State-Space Design: A Comprehensive Guide](https://blog.control-theory.com/entry/state-feedback-control-eng)

## Contents

| Folder | Blog Article | Hub Section | Description |
|--------|-------------|-------------|-------------|
| `01_pole_placement/` | [State Feedback Control: Pole Placement](https://blog.control-theory.com/entry/2024/10/01/143612) | State Feedback: Pole Placement | Step response comparison before/after pole placement, pole map on the s-plane, effect of pole locations on transient response |
| `02_lqr_design/` | [State Feedback Control: Optimal Regulators](https://blog.control-theory.com/entry/2024/10/01/143503) | State Feedback: Optimal Regulator (LQR) | LQR responses for different Q/R weights, state vs. control input trade-off, Riccati equation convergence |
| `03_integral_servo/` | *(Hub section only — no separate spoke article)* | Integral-Type Servo System | Step tracking with integral action, comparison of steady-state error with/without integrator |
| `04_observer_based_feedback/` | [State Observer for State Space Model](https://blog.control-theory.com/entry/2024/10/01/143305) | Observer-Based Feedback and the Separation Principle | Separation principle demo using the same plant as `01`–`03` (lightweight version — see [Related Repositories](#related-repositories)) |
| `05_lmi_state_feedback/` | [LMI and Controller Design](https://blog.control-theory.com/entry/2024/10/19/170411) / [Advanced LMI Techniques](https://blog.control-theory.com/entry/2025/04/20/173054) | LMI-Based State Feedback Design | H∞ state feedback via LMI, comparison with pole placement and LQR |

## Repository Structure

Each folder contains both MATLAB and Python implementations with a shared `fig/` directory:

```
control_state_feedback/
├── README.md
├── 01_pole_placement/
│   ├── matlab/
│   │   └── pole_placement.m
│   ├── python/
│   │   └── pole_placement.py
│   └── fig/
├── 02_lqr_design/
│   ├── matlab/
│   │   └── lqr_design.m
│   ├── python/
│   │   └── lqr_design.py
│   └── fig/
├── 03_integral_servo/
│   ├── matlab/
│   │   └── integral_servo.m
│   ├── python/
│   │   └── integral_servo.py
│   └── fig/
├── 04_observer_based_feedback/
│   ├── matlab/
│   │   └── observer_based_feedback.m
│   ├── python/
│   │   └── observer_based_feedback.py
│   └── fig/
└── 05_lmi_state_feedback/
    ├── matlab/
    │   └── lmi_state_feedback.m
    ├── python/
    │   └── lmi_state_feedback.py
    └── fig/
```

## Requirements

### MATLAB

- MATLAB R2023a or later (recommended)
- Control System Toolbox
- Robust Control Toolbox (for `05_lmi_state_feedback/` only)

### Python

```
pip install numpy scipy matplotlib control
```

For `05_lmi_state_feedback/` only:

```
pip install cvxpy
```

## Running the Simulations

Each script is self-contained and can be executed independently. The `fig/` directory is created automatically.

**MATLAB:**

```matlab
cd 01_pole_placement/matlab
pole_placement
```

**Python:**

```bash
cd 01_pole_placement/python
python pole_placement.py
```

## Plot Style

All figures follow a unified plot style for consistency across the blog. Style definitions (colors, figure sizes, fonts, and save functions) are embedded directly in each script — no external dependencies or shared modules are required. Key conventions:

- **Figure size**: (8, 5) inches at 150 dpi for standard time-series plots
- **Font**: Serif, 13 pt (axes labels 14 pt)
- **Colors**: Black for reference/true values, blue (`#1f77b4`) for method 1, red (`#d62728`) for method 2, green (`#2ca02c`) for method 3, purple (`#9467bd`) for control input
- **Background**: White (`facecolor='white'`) for blog compatibility

## Related Repositories

This repository focuses on **state feedback design** (pole placement, LQR, integral servo) and provides a lightweight separation-principle demo in `04_observer_based_feedback/`.

For comprehensive **observer design** — including Luenberger observer, Kalman filter, H∞ filter, multi-rate observer, and outlier-robust MCV observer — see:

- **MATLAB**: [MATLAB_state_observer](https://github.com/Hiroshi-Okajima/MATLAB_state_observer)
- **Python**: [python_state_observer](https://github.com/Hiroshi-Okajima/python_state_observer)
- **Blog hub**: [State Observer and State Estimation: A Comprehensive Guide](https://blog.control-theory.com/entry/state-observer-estimation)

For **model-based compensation** (IMC, DOB, 2-DOF, MEC) and comparison studies, see:

- [control_model_based_compensation](https://github.com/Hiroshi-Okajima/control_model_based_compensation)
- **Blog hub**: [Model Error Compensator (MEC)](https://blog.control-theory.com/entry/model-error-compensator-eng)

## License

MIT License
