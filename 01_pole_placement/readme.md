# 01_pole_placement — State Feedback: Pole Placement

## Blog Article

- **Spoke**: [State Feedback Control: Design Gains using Pole Placement](https://blog.control-theory.com/entry/2024/10/01/143612)
- **Hub**: [State Feedback Control and State-Space Design: A Comprehensive Guide](https://blog.control-theory.com/entry/state-feedback-control-eng)

## Description

This script demonstrates **state feedback pole placement** for a 3rd-order
unstable system in controllable canonical form. Four different pole
placements are compared to illustrate the effect of pole locations on
convergence speed, oscillation, and control effort.

## Plant Model

3rd-order controllable canonical form (from the blog article):

```
     [0  1  0]       [0]
A =  [0  0  1],  B = [0],  C = [1  0  0],  D = 0
     [-1 1  1]       [1]
```

**Open-loop poles**: 1 (double root), −1 → **unstable**

## Feedback Designs

| Design | Desired poles       | Feedback gain K       | Characteristics           |
|--------|--------------------|-----------------------|---------------------------|
| 1      | {−1, −2, −3}       | [5, 12, 7]            | Moderate (blog example)   |
| 2      | {−0.5, −1, −2}     | (computed by acker)   | Slow — poles near Im axis |
| 3      | {−5, −6, −7}       | (computed by acker)   | Fast — poles far left     |
| 4      | {−2, −1±5j}        | (computed by acker)   | Oscillatory — large Im    |

## Generated Figures

| File | Description |
|------|-------------|
| `fig/pole_placement_step_response.png` | State response & control input: effect of real-part pole locations (Designs 1–3) |
| `fig/pole_placement_pole_map.png` | Pole map on the s-plane (all 4 designs + open-loop) |
| `fig/pole_placement_imaginary_effect.png` | Effect of imaginary-part pole locations (Design 1 vs 4) |

## Usage

### Python

```bash
cd 01_pole_placement/python
pip install numpy scipy matplotlib control
python pole_placement.py
```

### MATLAB

```matlab
cd 01_pole_placement/matlab
pole_placement
```

Figures are saved to `fig/` (Python) or `fig/` (MATLAB — create the directory first).
