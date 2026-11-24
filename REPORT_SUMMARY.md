# Project Report Summary — Cropped Delta Wing UAV Trim & Performance

## Key Findings (`α = 0°` to `12°` Primary Operating Envelope)

| Metric | Primary Sweep (`0:0.5:12°`) | Closed-Form Theory | Operating Condition |
|---|---:|---:|---|
| **Minimum Thrust Required ($T_{r,\min}$)** | **4.2901 N** | 4.2897 N | $\alpha = 10.0^\circ$, $\delta_e = -5.8676^\circ$, $V = 12.263\text{ m/s}$, $C_L = 0.48250$ |
| **Maximum Lift-to-Drag Ratio ($(L/D)_{\max}$)** | **8.1583** | 8.1592 | $\alpha = 10.0^\circ$, $V = 12.263\text{ m/s}$ *(coincides with $T_{r,\min}$)* |
| **Minimum Power Required ($P_{r,\min}$, `0–12°`)** | **48.737 W** | 45.8215 W *(unconstrained)* | Sweep bound $\alpha = 12.0^\circ$, $\delta_e = -7.3276^\circ$, $V = 11.208\text{ m/s}$, $C_L^{3/2}/C_D = 6.1174$ |
| **Minimum Power Required (Extended `-2–20°`, no stall)** | **45.823 W** *(at $17.5^\circ$)* | 45.8215 W *(at $17.68^\circ$)* | Indicative linear model: $C_L \approx 0.848$, $V \approx 9.25\text{ m/s}$, $C_L^{3/2}/C_D \approx 6.507$ |
| **Trim Elevator Deflection Range (`0–12°`)** | **-7.3276° to +1.4324°** | — | $+1.4324^\circ$ (nose-down/trailing-edge-down at $0^\circ$) to $-7.3276^\circ$ (at $12^\circ$) |
| **Level-Flight Velocity Range (`0–12°`)** | **11.208 to 104.656 m/s** | — | $11.208\text{ m/s}$ at $12^\circ$ ($C_L = 0.5777$) to $104.656\text{ m/s}$ at $0^\circ$ ($C_L = 0.00663$); $11.21\text{–}26.70\text{ m/s}$ for $\alpha \in [2^\circ, 12^\circ]$ |

### Summary Highlights
- **Derived Wing Properties:** Taper ratio $\lambda = 0.1667$, planform area $S = 0.7875\text{ m}^2$, aspect ratio $AR = 2.8571$, induced drag factor $k = 0.1252$, weight $W = 35.00\text{ N}$.
- **Static Pitch Stability & Trim:** Because $C_{m\alpha} = -0.292\text{ rad}^{-1} < 0$, the wing is statically stable in pitch; increasing $\alpha$ requires increasingly negative (trailing-edge-up) elevator deflection $\delta_e$ from $+1.432^\circ$ at $\alpha = 0^\circ$ to $-7.328^\circ$ at $\alpha = 12^\circ$.
- **Minimum Thrust & Maximum Endurance:** Minimum thrust (`4.2901 N`) and maximum aerodynamic efficiency ($C_L/C_D = 8.1583$) both occur at $\alpha = 10.0^\circ$ ($V = 12.26\text{ m/s}$), matching closed-form parabolic polar theory within `0.01%`.
- **Minimum Power Behavior:** Within `0°–12°`, power required decreases monotonically to `48.737 W` at $\alpha = 12.0^\circ$ ($V = 11.21\text{ m/s}$). Extending the linear model to `-2°–20°` reveals the unconstrained minimum power point (`45.823 W`) near $\alpha = 17.5^\circ$ ($C_L \approx 0.84\text{–}0.85$), though results above $\sim 12^\circ$ are indicative only because stall is unmodeled.

---

## Full Results Table (`α = 0:0.5:12` deg)

Exported identically to [uav_trim_results.csv](uav_trim_results.csv) and [uav_trim_results.tex](uav_trim_results.tex).

| $\alpha$ (deg) | $\delta_e$ (deg) | $C_L$ (-) | $C_D$ (-) | $V$ (m/s) | $T_r$ (N) | $P_r$ (W) | $C_L/C_D$ (-) | $C_L^{3/2}/C_D$ (-) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.0 | 1.4324 | 0.00663 | 0.03001 | 104.656 | 158.5196 | 16589.974 | 0.2208 | 0.0180 |
| 0.5 | 1.0674 | 0.03042 | 0.03012 | 48.841 | 34.6516 | 1692.423 | 1.0101 | 0.1762 |
| 1.0 | 0.7024 | 0.05421 | 0.03037 | 36.585 | 19.6058 | 717.285 | 1.7852 | 0.4157 |
| 1.5 | 0.3374 | 0.07801 | 0.03076 | 30.499 | 13.8023 | 420.962 | 2.5358 | 0.7082 |
| 2.0 | -0.0276 | 0.10180 | 0.03130 | 26.698 | 10.7604 | 287.284 | 3.2527 | 1.0378 |
| 2.5 | -0.3926 | 0.12559 | 0.03197 | 24.037 | 8.9106 | 214.180 | 3.9279 | 1.3920 |
| 3.0 | -0.7576 | 0.14939 | 0.03279 | 22.039 | 7.6832 | 169.334 | 4.5554 | 1.7607 |
| 3.5 | -1.1226 | 0.17318 | 0.03375 | 20.469 | 6.8218 | 139.638 | 5.1306 | 2.1351 |
| 4.0 | -1.4876 | 0.19697 | 0.03486 | 19.193 | 6.1936 | 118.877 | 5.6510 | 2.5080 |
| 4.5 | -1.8526 | 0.22077 | 0.03610 | 18.130 | 5.7234 | 103.762 | 6.1153 | 2.8733 |
| 5.0 | -2.2176 | 0.24456 | 0.03749 | 17.225 | 5.3649 | 92.411 | 6.5239 | 3.2263 |
| 5.5 | -2.5826 | 0.26836 | 0.03901 | 16.444 | 5.0885 | 83.673 | 6.8783 | 3.5632 |
| 6.0 | -2.9476 | 0.29215 | 0.04068 | 15.760 | 4.8740 | 76.814 | 7.1809 | 3.8813 |
| 6.5 | -3.3126 | 0.31594 | 0.04250 | 15.155 | 4.7076 | 71.343 | 7.4348 | 4.1790 |
| 7.0 | -3.6776 | 0.33974 | 0.04445 | 14.615 | 4.5791 | 66.921 | 7.6434 | 4.4551 |
| 7.5 | -4.0426 | 0.36353 | 0.04654 | 14.128 | 4.4811 | 63.309 | 7.8107 | 4.7093 |
| 8.0 | -4.4076 | 0.38732 | 0.04878 | 13.687 | 4.4079 | 60.332 | 7.9403 | 4.9417 |
| 8.5 | -4.7726 | 0.41112 | 0.05116 | 13.285 | 4.3552 | 57.861 | 8.0363 | 5.1528 |
| 9.0 | -5.1376 | 0.43491 | 0.05368 | 12.917 | 4.3197 | 55.797 | 8.1024 | 5.3433 |
| 9.5 | -5.5026 | 0.45870 | 0.05634 | 12.577 | 4.2987 | 54.067 | 8.1419 | 5.5143 |
| **10.0** | **-5.8676** | **0.48250** | **0.05914** | **12.263** | **4.2901** | **52.611** | **8.1583** | **5.6669** |
| 10.5 | -6.2326 | 0.50629 | 0.06209 | 11.972 | 4.2921 | 51.383 | 8.1545 | 5.8023 |
| 11.0 | -6.5976 | 0.53009 | 0.06517 | 11.700 | 4.3032 | 50.348 | 8.1334 | 5.9217 |
| 11.5 | -6.9626 | 0.55388 | 0.06840 | 11.446 | 4.3224 | 49.473 | 8.0974 | 6.0263 |
| **12.0** | **-7.3276** | **0.57767** | **0.07177** | **11.208** | **4.3486** | **48.737** | **8.0486** | **6.1174** |
