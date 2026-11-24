# Cropped Delta Wing UAV — Steady Level Flight Trim & Performance Analysis

Longitudinal trim, thrust-required, and power-required analysis for a wing-alone cropped delta UAV in steady, unaccelerated level flight ($L = W$, $T_r = D$, $C_m = 0$). Written in modular, struct-based `.m` scripts compatible with both **MATLAB** and **GNU Octave**, accompanied by closed-form validation tests, an Excel workbook with live formulas/charts, and publication-quality 300 dpi figures.

---

## Folder Structure

```text
bserc-internship/
├── main_run.m                     # Main driver (0..12 deg primary, -2..20 deg extended, V-solver)
├── compute_geometry.m             # Planform taper ratio, area, aspect ratio, induced drag factor, weight
├── trim_solver.m                  # Solves Cm = 0 for trim elevator deflection & trimmed CL
├── performance.m                  # Computes V, CD, Tr, Pr, CL/CD, CL^(3/2)/CD, and extrema
├── velocity_trim_solver.m         # Iterative fzero solver on lift equilibrium L = W given airspeed V
├── plot_results.m                 # 3x3 summary figure (with optional indicative/overlay markers)
├── export_publication_figures.m   # Exports 8 separate 300-dpi PNG figures into figures/
├── validate_model.m               # Assert-based unit tests & closed-form vs. sweep pass/fail tables
├── script.m                       # Original monolithic reference script
├── uav_trim.xlsx                  # Excel workbook with live formulas (Inputs, Calculations, Charts)
├── uav_trim_results.csv           # Full numerical results table for alpha = 0:0.5:12 deg
├── uav_trim_results.tex           # LaTeX booktabs table for alpha = 0:0.5:12 deg
├── REPORT_SUMMARY.md              # Key findings summary & full Markdown results table
├── README.md                      # Project documentation
└── figures/                       # 8 separate publication-quality 300-dpi PNG plots
    ├── 01_trim_alpha_vs_V.png
    ├── 02_trim_elevator_vs_alpha.png
    ├── 03_CL_vs_alpha.png
    ├── 04_CD_vs_alpha.png
    ├── 05_thrust_required_vs_V.png
    ├── 06_power_required_vs_V.png
    ├── 07_CL_CD_vs_alpha.png
    └── 08_CL32_CD_vs_alpha.png
```

---

## Input Parameters & Governing Equations

| Parameter | Symbol | Value | Units |
|---|---|---:|---|
| Root chord | $C_r$ | 0.90 | m |
| Tip chord | $C_t$ | 0.15 | m |
| Wingspan | $b$ | 1.50 | m |
| Oswald efficiency | $e$ | 0.89 | — |
| Zero-lift drag coefficient | $C_{D0}$ | 0.03 | — |
| Zero-$\alpha$ pitching moment | $C_{m0}$ | 0.01 | — |
| Zero-$\alpha$ lift coefficient | $C_{L0}$ | 0.00 | — *(assumed)* |
| Lift-curve slope | $C_{L\alpha}$ | 2.92 | rad⁻¹ |
| Elevator lift effectiveness | $C_{L\delta_e}$ | 0.265 | rad⁻¹ |
| Pitching moment slope | $C_{m\alpha}$ | -0.292 | rad⁻¹ |
| Elevator control power | $C_{m\delta_e}$ | -0.40 | rad⁻¹ |
| Aircraft mass | $m$ | 3.50 | kg |
| Air density | $\rho$ | 1.225 | kg/m³ |
| Gravitational acceleration | $g$ | 10.0 | m/s² |

- **Geometry & Weight:** $\lambda = C_t/C_r$, $S = \frac{b}{2}C_r(1+\lambda)$, $AR = b^2/S$, $k = 1/(\pi e AR)$, $W = mg$
- **Pitch Trim ($C_m = 0$):** $\delta_{e,\text{trim}} = -\dfrac{C_{m0} + C_{m\alpha}\alpha}{C_{m\delta_e}}$, $C_L = C_{L0} + C_{L\alpha}\alpha + C_{L\delta_e}\delta_{e,\text{trim}}$
- **Level Flight Performance ($L = W$, $T_r = D$):** $V = \sqrt{\dfrac{2W}{\rho S C_L}}$, $C_D = C_{D0} + k C_L^2$, $T_r = \frac{1}{2}\rho V^2 S C_D$, $P_r = T_r V$

---

## How to Run

Open **MATLAB** or **GNU Octave** in the repository root folder and run:

```matlab
% Run the full analysis (primary 0..12 deg sweep, -2..20 deg extended sweep,
% iterative velocity solver for 8..30 m/s, CSV exports, and PNG figures)
main_run

% Run assert-based unit tests and closed-form drag-polar validation tables
report = validate_model();
```

In headless GNU Octave CLI:

```bash
octave-cli --eval "main_run; validate_model;"
```

---

## Modeling Assumptions

1. **$C_{L0} = 0$ (Assumed):** Zero-angle-of-attack lift coefficient was not specified in the problem statement, so it is assumed to be `0` and exposed explicitly as `params.CL0` in `main_run.m` and `Inputs!C10` in `uav_trim.xlsx`.
2. **Linear Aerodynamics:** $C_L$ and $C_m$ are modeled as linear functions of $\alpha$ and $\delta_e$ with constant stability and control derivatives across the entire sweep.
3. **No Stall Model:** Flow separation / $C_{L,\max}$ is not modeled. Accordingly, `0° ≤ α ≤ 12°` is kept as the primary operating deliverable; results beyond ~`12°` in the extended `-2°` to `20°` case (including the unconstrained minimum-power point near `17.7°`) are indicative only.
4. **Sea-Level Atmospheric & Mass Constants:** Standard sea-level air density $\rho = 1.225\text{ kg/m}^3$ and $g = 10.0\text{ m/s}^2$ are assumed constant.
5. **Steady Level Flight & Parabolic Polar:** Wing-alone configuration with $L = W$ (points where $C_L \le 0$ cannot sustain level flight and are guarded as `NaN`), $T_r = D$, and $C_D = C_{D0} + k C_L^2$.

---

## Expected Output

Running `main_run` produces:
- Console printout of derived geometry (`lambda = 0.1667`, `S = 0.7875 m^2`, `AR = 2.8571`, `k = 0.1252`, `W = 35.00 N`), full `0:0.5:12` deg table, extended `-2:0.5:20` deg extrema, and `V = 8:0.5:30` m/s iterative `fzero` table (matching the $\alpha$-sweep to `< 1e-8`).
- CSV files: `uav_trim_results.csv`, `uav_trim_results_extended.csv`, `uav_velocity_trim_results.csv`.
- Summary figures (`uav_trim_plots.png`, `uav_trim_plots_extended.png`) and 8 separate 300-dpi plots in `figures/`.

Running `validate_model` executes:
- `assert` unit tests for `S`, `AR`, `k`, `lambda`, `W`, $C_m = 0$ at every trimmed point (`max |Cm| < 1e-12`), and iterative velocity-solver residuals (`max |L - W| < 1e-8`).
- Pass/fail percentage-error tables comparing numerical sweep extrema against closed-form parabolic drag-polar theory.
