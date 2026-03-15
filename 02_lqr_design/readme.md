# 02_lqr_design — State Feedback: Optimal Regulator (LQR)

## Blog Article

- **Spoke**: [State Feedback Control: Design Gains using Optimal Regulators](https://blog.control-theory.com/entry/2024/10/01/143503)
- **Hub**: [State Feedback Control and State-Space Design: A Comprehensive Guide](https://blog.control-theory.com/entry/state-feedback-control-eng)

## Description

This script demonstrates **LQR (Linear Quadratic Regulator)** design for the
same 3rd-order plant used in `01_pole_placement/`. The weight matrix Q is fixed
at the identity, and the scalar weight R is varied across three values to
illustrate the trade-off between fast convergence and low control effort.

## Plant Model

Same 3rd-order controllable canonical form as `01_pole_placement/`:

```
     [0  1  0]       [0]
A =  [0  0  1],  B = [0],  C = [1  0  0],  D = 0
     [-1 1  1]       [1]
```

**Open-loop poles**: 1 (double root), −1 → **unstable**

## LQR Designs

| Design  | Q   | R     | Characteristics                  |
|---------|-----|-------|----------------------------------|
| Fast    | I   | 0.01  | Quick convergence, large input   |
| Balanced| I   | 1     | Moderate trade-off               |
| Energy  | I   | 100   | Slow convergence, small input    |

## Generated Figures

| File | Description |
|------|-------------|
| `fig/lqr_design_R_comparison.png` | Output response & control input for R = 0.01, 1, 100 |
| `fig/lqr_design_state_variables.png` | All three state variables: R = 0.01 vs R = 100 |
| `fig/lqr_design_pole_map.png` | Pole map comparing LQR designs with pole placement from 01 |
| `fig/lqr_design_cost_tradeoff.png` | Cost function decomposition (state penalty vs input energy) as R varies |

## Usage

### Python

```bash
cd 02_lqr_design/python
pip install numpy scipy matplotlib control
python lqr_design.py
```

### MATLAB

```matlab
cd 02_lqr_design/matlab
lqr_design
```

Figures are saved to `02_lqr_design/fig/`.
