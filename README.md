# T-Motor F1404 KV4600 — Physics-Based Motor Model for SIL Simulation

A data-driven C++ motor model for drone flight simulation, built from publicly available manufacturer test data using physics and polynomial regression.

---

## 🚧 Work in Progress:
  - v1.0 (current): static LUT from datasheet stand test - no dynamics, no temperature effects, no motor-to-motor variation.
--

## Iteration Roadmap and publishing plan

| Version | What changes | Requires |
|---|---|---|
| v1.0 (current) STATIC | Static LUT from manufacturer datasheet | Octave + datasheet |
| v2.0 DYNAMIC | Rotor inertia + back-EMF dynamics | Oscilloscope + test bench |
| v3.0 HIL | Eddy current + temperature + commutation noise | Full drone on tether |

> v1.0 is a static model based on the datasheet stand test.
> v2.0 measurements planned on physical test bench.
> v3.0 measurements planned on a real drone in a controlled environment (tethered test rig).
> Results will be published on [meugenom.com](https://meugenom.com) as open-source data and code.


## Table of Contents
- [Iteration Roadmap](#iteration-roadmap-and-publishing-plan)
- [What This Is](#what-this-is)
- [Project Documentation](#project-documentation)
- [How the Model Works](#how-the-model-works)
- [Code Pipeline](#code-pipeline)
- [Build & Test](#build--test)
- [Regenerating the Model](#regenerating-the-model-octave)
- [Usage in a Simulator](#usage-in-a-simulator)
- [Known Limitations](#known-limitations)
- [Model Specifications](#model-specifications)
- [Model Accuracy](#model-accuracy)
- [What This Model Does Not Yet Capture](#what-this-model-does-not-yet-capture)
- [References](#references)
- [License](#license)


## What This Is

This project models the thrust and current output of the **T-Motor F1404 KV4600** brushless motor with a **GF3016** propeller as a function of throttle position and battery voltage.

The model is intended for use in drone simulators where realistic motor behavior is required — including voltage sag effects, hover point estimation, and propeller load characteristics.

**Data source:** [T-Motor F1404 KV4600 datasheet stand test](https://n-factory.de/T-Motor-F1404-4600KV-Ultra-Light-Motor)  
11 measurement points (50–100% throttle) collected by the manufacturer at ~16V (4S LiPo), 8°C ambient, with a GF3016 propeller. A forced zero-point (0%, 0 RPM) is added before fitting, giving 12 data points total.

---

## Project Documentation

| File | Description |
|------|-------------|
| [MOTOR_SPEC.md](./MOTOR_SPEC.md) | Motor specifications and raw datasheet test report |
| [MOTOR_CALC.md](./MOTOR_CALC.md) | Model derivation: math, pipeline, voltage scaling |

---

## How the Model Works

All computation in `src/motor.cpp` is **table lookup + linear interpolation**, no math at runtime.

### Why V_nominal = 16.0V

The stand test was conducted at a slightly discharged 4S LiPo: voltage varied from 15.93V (50% throttle) to 15.64V (100% throttle) as current increased. To build a consistent lookup table, all RPM and current values are first normalised to a single reference voltage:

```
drehzahl_norm = drehzahl_upm × (16.0 / V_measured)
strom_norm    = strom_meas   × (16.0 / V_measured)²
```

16.0V was chosen as the nominal reference — close to the actual test conditions (within 0.25V) and a convenient round number for a fresh 4S LiPo (4× 4.0V). The normalisation error is less than 1.5%.

**Why RPM normalisation matters for the thrust polynomial:**

Without normalisation, `polyfit(drehzahl_upm, schub_g)` sees measurements taken at different voltages as if they were at the same voltage. The resulting polynomial is geometrically incorrect — it produces a concave-down curve instead of the physically correct concave-up parabola F ∝ n². After normalising all RPM values to 16V, the polynomial fits a proper F = k·n² relationship through a consistent physical space.

### Octave (`src/motor_model.m`) — offline preprocessing

The Octave script does all the physics once and bakes the results into arrays:

1. **RPM normalisation:** `drehzahl_norm = drehzahl_upm × (16.0 / V_measured)` — all RPM values brought to 16V reference.
2. **Polynomial fit (RPM → Thrust):** `polyfit(drehzahl_norm, schub_g, 2)` on 12 points gives coefficients A, B, C.
3. **Polynomial fit (Gas → RPM):** separate `polyfit` on the same normalised values.
4. **Polynomial fit (Gas → Current):** currents normalised via `I_norm = I_meas × (V_meas/16)²`, then `polyfit` on 12 points. This ensures `MOTOR_TAB_STROM` represents true 16V operating conditions.
5. **Evaluate all polynomials** on a uniform 101-point grid (0.00–1.00, step 0.01) to produce `MOTOR_TAB_SCHUB_N` (in Newtons), `MOTOR_TAB_DREHZAHL`, `MOTOR_TAB_STROM`.
6. **Hover throttle calculation:** solve the thrust equation for 62.5 g/motor, interpolate back to throttle via `MOTOR_TAB_DREHZAHL`.
7. **Export** everything to `includes/motor_lut.h`.

The polynomials and intermediate RPM values are **not used at runtime** — they only exist inside the Octave script.

### C++ (`src/motor.cpp`) — runtime

Both public functions do the same thing: O(1) table lookup + linear interpolation + voltage scaling.

**Thrust** scales with $V^2$ because $F \propto RPM^2 \propto V^2$ (KV law under propeller load):

$$F[N] = MOTOR\_TAB\_SCHUB\_N(throttle) \cdot \left(\frac{V}{V_{nom}}\right)^2$$

The conversion from grams to Newtons (`× 9.81 / 1000`) is done once in Octave during table generation — not at runtime.

**Current** scales with $V^2$ because $I \propto \omega^2 \propto V^2$ (propeller drag torque $M_{prop} \propto \omega^2$, motor torque $I = M/k_T$):

$$I[A] = MOTOR\_TAB\_STROM(throttle) \cdot \left(\frac{V}{V_{nom}}\right)^2$$

---

## Code Pipeline

```text
  src/motor_model.m             includes/motor_lut.h          src/motor.cpp
  ──────────────────            ────────────────────          ──────────────────
  Octave script         ──►     Auto-generated         ──►    C++ runtime
  - RPM normalise               C++ header                    - tabInterp O(1)
  - polyfit                     (DO NOT EDIT)                 - V² thrust scaling
  - bake to table                                             - V² current scaling
  - g→N conversion                                            - no math, no floats
  - hover calc
```

`includes/motor_lut.h` is fully auto-generated by `src/motor_model.m` and must never be edited manually.

### Workflow (end-to-end)

1. `src/motor_model.m` normalises RPM, fits polynomials, converts to Newtons, exports lookup-table constants.
2. `includes/motor_lut.h` is generated automatically from the Octave script.
3. `src/motor.cpp` compiles the runtime model used by simulators.
4. `src/test_motor.cpp` validates thrust/voltage behavior against datasheet values.

---

## Build & Test

The model uses a 2nd-degree polynomial fit, which provides a smooth transition between points. While it introduces a small mathematical deviation (approx. 5%) compared to raw datasheet values, it prevents step-response artifacts in PID controllers during simulation.

```sh
cd build && rm -rf * && cmake ..
make -j$(sysctl -n hw.ncpu)
./test_motor
```

Expected output (all green):

```text
=== Motor Model Tests: F1404 KV4600 ===
✓ zero throttle → zero thrust
✓ thrust at 50%: 1.874N (expected 1.807N)
✓ thrust at 75%: 2.718N (expected 2.821N)
✓ thrust at 100%: 3.303N (expected 3.382N)
✓ thrust is monotonically increasing
✓ lower voltage → lower thrust
✓ hover 250g quad: throttle=17.7%, total=247.4g (need 250g)

--- Stromtests (I ∝ V²) ---
✓ current at 50%: 5.74A (expected 5.23A)
✓ current at 75%: 11.00A (expected 11.32A)
✓ current at 100%: 17.85A (expected 17.54A)
✓ current is monotonically increasing
✓ current V²-scaling: I(16.8V)/I(14.0V) = 1.440 (expected 1.440)

✓ All tests passed.
Model accuracy (50/75/100%): 3.7% / 3.7% / 2.3% | avg=3.2%
```
---

## Regenerating the Model (GNU Octave)

If the propeller or motor changes, update the data in `motor_model.m` and re-run:

```sh
  cd src && octave ./motor_model.m
```
This overwrites `includes/motor_lut.h` with new precomputed tables and hover constants. Then rebuild the C++ project.
> **Note:** Run from `src/` directory — the script uses relative paths to `../includes/` and `../plots/`.

---

## Usage in a Simulator

Copy `src/motor.cpp`, `includes/motor.h`, `includes/motor_lut.h` into your project, or use CMake:

```cmake
add_subdirectory(path/to/tmotor-f1404-model)
target_link_libraries(your_target PRIVATE motor_model)
```

```cpp
#include "motor.h"

float thrustN  = getMotorThrustNewtons(0.75f, 15.8f);  // throttle, voltage
float currentA = getMotorCurrentAmps(0.75f, 15.8f);
```

---

## Known Limitations

### Low-throttle region (0–49%)

No stand test data exists below 50% throttle.
The 101-point tables are generated by a polynomial fit that includes a forced zero at (0%, 0 RPM, 0 A, 0 g thrust).
The fit is smooth but not validated below 50% — treat results in this region as an estimate.
The hover point at ~17.7% falls here.

### Voltage scaling

Both thrust and current use $V^2$ scaling — derived from $\omega \propto V$ (KV law) and the respective quadratic dependence on angular velocity. Valid for ±20% deviation from 16V (i.e., 12.8–19.2V). In practice, the 4S operating range is **14.0–16.8V**.

---

## Model Specifications

| Parameter | Value |
|---|---|
| Motor | T-Motor F1404 KV4600 |
| Propeller | GF3016 |
| KV | 4600 RPM/V |
| Internal resistance | 138 mΩ |
| Peak current (60s) | 20 A |
| Idle current (10V) | 0.6 A |
| Weight | 9.34 g |
| Rated voltage | 3–4S LiPo |
| V_nominal (model) | 16.0V |

---

## Model Accuracy

| Throttle | Thrust Error | Current Error |
|----------|-------------|---------------|
| 50%      | +3.7%       | +9.7%         |
| 75%      | -3.7%       | -2.8%         |
| 100%     | -2.3%       | +1.8%         |
| **avg**  | **3.2%**    | **4.8%**      |

Tolerance: ±5% thrust, ±10% current.

| Region | Status |
|---|---|
| 50–100% throttle, ~16V | Measured data, avg error 3.2% |
| 0–49% throttle | Polynomial extrapolation — no measurements |
| Voltage scaling | Valid for 14.0–16.8V (4S operating range) |
| Temperature effects | Not modelled |
| Motor-to-motor variation | Not modelled (~2–3% in practice) |

**Known limitation:** current model less accurate below 60% throttle (+9.7% at 50%).

---

## What This Model Does Not Yet Capture

Physical phenomena intentionally excluded from v1.0.
Planned for future iterations with real hardware measurements.

### Rotor Inertia

The model assumes instantaneous throttle response.
In reality the rotor has angular momentum — it cannot change speed instantly.

Time constant τ from motor parameters:

$$\tau = \frac{J \cdot R}{k_e \cdot k_T}$$

For F1404 geometry (rotor ~6 g, r ≈ 9 mm): τ ≈ 5–15 ms.

**Impact on simulation:** PID tuned on this model will be optimistic.
Real step response is slower than predicted.

### Back-EMF Dynamics

Static model uses $\omega \propto V$ (KV law).
Transient back-EMF interaction is not captured:

$$V_{eff}(t) = V_{supply} - k_e \cdot \omega(t)$$

where $k_e = \frac{60}{2\pi \cdot KV} = 0.00208 \; \text{V·s/rad}$

Affects current prediction accuracy during rapid throttle changes.

### Eddy Current Losses

At high electrical frequencies, eddy currents in the stator core cause losses beyond DC winding resistance.

Electrical frequency at maximum RPM (9N12P motor, 6 pole pairs):

$$f_{elec} = \frac{40053 \cdot 6}{60} \approx 4005 \; \text{Hz}$$

At ~4 kHz, core losses become significant and explain part of the gap between theoretical and measured efficiency.

### Commutation Noise

PWM switching (30–60 kHz) generates harmonic current spikes not visible in averaged datasheet values.

**System-level effects:**
- Magnetic field interference with IMU magnetometer
- Power supply ripple on flight controller
- EMC behavior of the complete drone system

### Temperature Effects

Copper winding resistance increases with temperature:

$$R(T) = R_{20°C} \cdot [1 + \alpha \cdot (T - 20°C)]$$

where α = 0.00393 /°C for copper.

At 60°C operating temperature: R increases ~16%, reducing current and thrust at constant throttle.

---

## References

1. [T-Motor F1404 KV4600 — Stand Test Data](https://n-factory.de/T-Motor-F1404-4600KV-Ultra-Light-Motor)  
   Manufacturer datasheet. Primary data source for this model.

2. "A Comparative Study on Thrust Map Estimation for Multirotor Aerial Vehicles", Francisco J. Anguita, Rafael Perez-Segui, Carmen DR.Pita-Romero, Miguel Fernandez-Cortizas, Javier Melero-Deza, Pascual Campoy, pages 100-105, 16th INTERNATIONAL MICRO AIR VEHICLE CONFERENCE AND COMPETITION, http://www.imavs.org.

3. "Modelling and Control of a Large Quadrotor Robot", P.Pounds, R.Mahony, P.Corke, September 2010, pages 1-26.

4. Propeller Performance Data at Low Reynolds Numbers, John B. Brandt and Michael S. Selig, AIAA 2011-1255, pages 1-18.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE)