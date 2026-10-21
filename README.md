# Autonomous Line Following on the T-Bot

Lab 4 of **Embedded Real-Time Control** — M.Sc. Control Systems Engineering, University of Padova
(course held by Prof. Alberto Morato, 2025).

`STM32F767` · `bare-metal C` · `STM32CubeIDE` · `SX1509 I²C` · `QTR reflectance array`

A differential-drive robot follows a high-contrast line on the floor using an eight-channel reflectance
array. The line position is estimated as a **weighted centroid**, turned into a heading error, and steered
with a proportional yaw-rate law that feeds the two **PI speed loops with anti-windup** built in Lab 3.

---

## Purpose

Make the T-Bot track a line autonomously. Sensing and actuation run in the same fixed-rate frame
(`TS = 10 ms`, `TIM6` period-elapsed interrupt): the sensor bank is read over I²C, the lateral position and
yaw error are computed, the two wheel speed references come from the differential-drive kinematics, and one
PI controller per wheel produces the PWM duty.

## Control law

| Step | Expression | Notes |
|---|---|---|
| Lateral position | `e_sl = Σ bᵢ·wᵢ / Σ bᵢ` | `bᵢ` = sensor *i* sees the line, `wᵢ = ((N−1)/2 − i)·P` |
| Array geometry | `N = 8`, `P = 8 mm` | pitch from the QTR datasheet |
| Yaw error | `ψ̂ = atan2(e_sl, H)` | `H = 0.085 m`, sensor array to vehicle centre |
| Yaw-rate command | `ψ̇ = k_ψ·ψ̂` | `k_ψ = 5` |
| Wheel references | `v_r = v + ψ̇·D/2`, `v_l = v − ψ̇·D/2` | `D = 0.165 m` wheelbase |
| Base speed | `v = 0.534 m/s` straight, `0.66·v` if `|ψ̂| ≥ 0.087 rad` (5°) | ≈ 150 rpm at the wheel |
| Wheel speed loop | PI, `P = 0.8`, `I = 7.5563`, `TS = 10 ms` | conditional-integration anti-windup, output clamped to ±6 V |

## What I implemented

- **Sensor readout in the control ISR** — the reflectance bank is read over I²C (`REG_DATA_B` of the SX1509
  expander at `0x3E`) inside the `TIM6` callback, so sensing and actuation share one time base.
- **Bit-pattern decoding and noise rejection** — `StoreBinary()` turns the sensor byte into eight per-sensor
  flags; `CalculateEsl()` then drops isolated sensors (a lit sensor whose two neighbours are dark, with the
  matching end-of-array rule) before any arithmetic, so one noisy photodiode cannot bias the centroid.
- **Weighted-centroid lateral position** — each sensor is weighted by its offset from the array centre, and
  the position is normalised by the number of active sensors (division guarded for the all-dark case).
- **Yaw error from the lever arm** — the lateral offset is converted to a heading error with
  `atan2(e_sl, H)`, which keeps the command bounded and correct in sign on both sides of the line.
- **Steering plus adaptive speed** — proportional yaw-rate command, differential-drive wheel references, and
  a base-speed reduction to 66 % once the heading error passes 5°, so the robot slows into curves and keeps
  full speed on straights.
- **Two independent speed loops** — the wheel references are converted to rpm through the wheel radius
  (`r = 0.034 m`) and tracked by one PI + anti-windup controller per wheel; the volt-level output is scaled
  to PWM duty against the measured battery voltage and applied with the lab's `ForwardCoast` /
  `ForwardBreak` drive modes.
- **Encoder handling fixed for two motors** — the quadrature counters (TIM3/TIM4) are read with a *per-motor*
  previous count and explicit overflow/underflow correction, so the two wheels no longer share state
  (this was the main change between my `v1` and `v2` builds).
- **Instrumentation** — `printf` retargeted to ITM/SWO for live traces, plus the course datalogger streaming
  the control signals to MATLAB for tuning.

## Results

The robot follows the track on the real T-Bot: it holds the line on straights and slows into the curves, with
the direction correction coming entirely from the sensor centroid — no other feedback is used. **This lab had
no written report to submit**, unlike Labs 1–3, so this repository is the record of the work: the extracted
source, the [one-page overview](docs/project-overview.pdf) and the control law above.

What the approach does not handle: the centroid collapses when the line leaves the array entirely (no
recovery / search behaviour), and the fixed steering gain means the transient differs between a sharp corner
and a gentle bend.

## Repository contents

| Path | What it is |
|---|---|
| `src/user_code_main.c` | The `USER CODE` sections of the `main.c` I submitted — the only part of that file that is mine (sensing, estimation, control, drive, ISR) |
| `docs/project-overview.pdf` | One-page overview of the lab, implementation and results (`.tex` source included) |
| `README.md` | This file |

## How to build

The project was built in **STM32CubeIDE** for the course T-Bot board (STM32F767ZI). The CubeMX-generated
skeleton, the HAL, the course datalogger (`ertc-datalogger.h`) and the `SX1509_Registers.h` helper belong to
the course and are **not redistributed here**, so this repository is source documentation rather than a
standalone buildable project. To rebuild it, create the lab base project on the course board and paste the
`USER CODE` blocks into the matching sections of `main.c`.

## Skills demonstrated

Reflectance-array line sensing over I²C · sensor-pattern filtering · weighted-centroid estimation ·
differential-drive kinematics and adaptive speed scheduling · cascaded control (steering → wheel speed) ·
fixed-rate real-time scheduling in a timer ISR · dual-motor quadrature encoder handling with overflow
correction · PI control with anti-windup · bare-metal C on ARM Cortex-M.

## Related repositories

- [`ertc-lab3-motor-control-pid`](https://github.com/Kiya-rb/ertc-lab3-motor-control-pid) — the PI + anti-windup
  motor speed control this lab builds on
- [`ertc-lab2-imu-pan-tilt-control`](https://github.com/Kiya-rb/ertc-lab2-imu-pan-tilt-control) — IMU-based
  pan/tilt control
- [`ertc-lab1-i2c-keypad-line-sensor`](https://github.com/Kiya-rb/ertc-lab1-i2c-keypad-line-sensor) — the
  SX1509 I²C driver and line-sensor readout reused here

## Licence

MIT — see [LICENSE](LICENSE). The course framework, HAL and vendor headers remain the property of their owners.
