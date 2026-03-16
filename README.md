# control_state_feedback

MATLAB and Python simulation code for state feedback control design. Companion code for [blog.control-theory.com](https://blog.control-theory.com/).

**Author**: Hiroshi Okajima, Kumamoto University  
**Blog Hub**: [State Feedback Control and State-Space Design: A Comprehensive Guide](https://blog.control-theory.com/entry/state-feedback-control-eng)

---

## Contents

| Folder | Topic | Blog Article | Description |
|--------|-------|--------------|-------------|
| `01_pole_placement/` | Pole Placement | [State Feedback Control: Pole Placement](https://blog.control-theory.com/entry/2024/10/01/143612) | Pole placement design and step response comparison with varying pole locations |
| `02_lqr_design/` | LQR Design | [State Feedback Control: Optimal Regulators](https://blog.control-theory.com/entry/2024/10/01/143503) | Linear Quadratic Regulator design with varying Q and R weights |
| `03_integral_servo/` | Integral Servo | Hub article only | Integral-type servo system for step tracking with zero steady-state error |
| `04_observer_based_feedback/` | Observer-Based Feedback | [State Observer for State Space Model](https://blog.control-theory.com/entry/2024/10/01/143305) | Separation principle demonstration using estimated states from observer |
| `05_lmi_state_feedback/` | LMI-Based Design | [LMI and Controller Design](https://blog.control-theory.com/entry/2024/10/19/170411) | H∞ state feedback design via Linear Matrix Inequality optimization |
| `common/` | — | — | Shared plot style and utility functions (self-contained in each script) |

---

## Requirements

### Python
```bash
python >= 3.10
numpy
scipy
matplotlib
control              # python-control library
cvxpy               # (Required for LMI-based design only)
```

Install dependencies:
```bash
pip install numpy scipy matplotlib control cvxpy
```

### MATLAB
```
MATLAB R2023a or later (recommended)
Control System Toolbox
Robust Control Toolbox (for LMI-based design only)
```

---

## Quick Start

### Python

```bash
cd 01_pole_placement
python pole_placement.py
```

This will generate PNG figures in `fig/` subdirectory.

### MATLAB

```matlab
cd 01_pole_placement
pole_placement
```

---

## Folder Details

### 01_pole_placement
Demonstrates pole placement technique for state feedback design.
- **Generated figures**: Step response comparison, pole map in s-plane
- **Plant model**: Second-order system (transfer function provided in script)
- **Design variations**: Multiple pole locations showing effect on response speed and damping

**Usage**:
```bash
python 01_pole_placement/pole_placement.py
matlab -r "cd('01_pole_placement'); pole_placement; exit"
```

---

### 02_lqr_design
Linear Quadratic Regulator (LQR) optimal control design.
- **Generated figures**: Step response for different Q/R weight combinations, cost trade-off analysis
- **Plant model**: Same second-order system as Folder 01
- **Design variations**: 3–4 LQR designs with varying Q and R penalty matrices

**Usage**:
```bash
python 02_lqr_design/lqr_design.py
```

---

### 03_integral_servo
Integral-type servo system for step tracking.
- **Generated figures**: Step tracking response (with/without integral action), comparison of steady-state behavior
- **Plant model**: Same second-order system as Folders 01–02
- **Key feature**: Demonstrates zero steady-state error for step reference tracking

**Usage**:
```bash
python 03_integral_servo/integral_servo.py
```

---

### 04_observer_based_feedback
Separation principle: combining state feedback (from Folder 01–03) with state observer.
- **Generated figures**: Closed-loop response using estimated states
- **Design approach**: Light-weight demonstration (details of observer design parameter tuning are deferred to [MATLAB_state_observer](https://github.com/Hiroshi-Okajima/MATLAB_state_observer))
- **Key concept**: Separation principle shows that observer gain L and feedback gain K can be designed independently

**Usage**:
```bash
python 04_observer_based_feedback/observer_based_feedback.py
```

**Note**: For comprehensive observer design techniques (Kalman filter, H∞ filter, multirate observer), see [MATLAB_state_observer](https://github.com/Hiroshi-Okajima/MATLAB_state_observer).

---

### 05_lmi_state_feedback
H∞ state feedback design using Linear Matrix Inequality (LMI) optimization.
- **Generated figures**: Closed-loop response, performance comparison with Pole Placement and LQR
- **Solver**: `cvxpy` for convex optimization
- **Design objective**: Robust performance under bounded uncertainty

**Usage**:
```bash
python 05_lmi_state_feedback/lmi_state_feedback.py
```

---

### common/
This folder contains shared style definitions and utilities.

**Note**: All plot style definitions (colors, figure sizes, fonts, save functions) are embedded **directly within each script** for stand-alone execution. No external imports of style modules are used. This self-contained approach ensures that any script can be run independently with:
```bash
python script_name.py
```

---

## Plot Style

All figures adhere to the unified plot style:

| Property | Value |
|----------|-------|
| **Figure size** | (8, 5) inches for standard plots; (8, 6) for Bode plots; (6, 6) for pole maps |
| **DPI** | 150 |
| **Font family** | Serif |
| **Font size** | 13 pt (base); 14 pt (axes labels); 12 pt (legend, ticks) |
| **Background** | White (`facecolor='white'`) |
| **Grid** | Enabled with α = 0.3 |
| **Line width** | 2.0 for reference; 1.5 for responses |

### Color Scheme

| Signal/Method | Color | Hex |
|---------------|-------|-----|
| Reference / True value | Black solid | `#000000` |
| Method 1 / Response | Blue | `#1f77b4` |
| Method 2 / Response | Red | `#d62728` |
| Method 3 / Response | Green | `#2ca02c` |
| Method 4 / Response | Orange | `#ff7f0e` |
| Control input | Purple | `#9467bd` |
| Baseline / No compensation | Gray dashed | `#7f7f7f` |

---

## Repository Structure

```
control_state_feedback/
├── README.md
├── 01_pole_placement/
│   ├── pole_placement.py
│   ├── pole_placement.m
│   └── fig/
│       ├── .gitkeep
├── 02_lqr_design/
│   ├── lqr_design.py
│   ├── lqr_design.m
│   └── fig/
│       ├── .gitkeep
├── 03_integral_servo/
│   ├── integral_servo.py
│   ├── integral_servo.m
│   └── fig/
│       ├── .gitkeep
├── 04_observer_based_feedback/
│   ├── observer_based_feedback.py
│   ├── observer_based_feedback.m
│   ├── README.md
│   └── fig/
│       ├── .gitkeep
├── 05_lmi_state_feedback/
│   ├── lmi_state_feedback.py
│   ├── lmi_state_feedback.m
│   └── fig/
│       ├── .gitkeep
└── common/
    └── (Style definitions are embedded in each script)
```

---

## Usage Notes

1. **Each script is self-contained**: All plot style definitions (colors, figure sizes, fonts) are embedded directly in each script. No external style modules or shared imports across scripts are used.

2. **Figure output**: PNG files are saved to `fig/` subdirectories automatically. If `fig/` does not exist, scripts will create it.

3. **Plant model parameters**: All parameters (system matrices, initial conditions, simulation time) are defined at the top of each script for easy modification.

4. **Python environment**: The `python-control` library ([`control`](https://python-control.readthedocs.io/)) is used for consistency with MATLAB control theory notation.

---

## License

MIT License

---

## Related Resources

- **Blog Hub**: [State Feedback Control and State-Space Design: A Comprehensive Guide](https://blog.control-theory.com/entry/state-feedback-control-eng)
- **Blog Site**: https://blog.control-theory.com/
- **Author**: [Hiroshi Okajima](https://github.com/Hiroshi-Okajima)
