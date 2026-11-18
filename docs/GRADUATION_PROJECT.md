# Graduation project: relationship to this repository

**Project:** "Autonomous Underwater Rocket Delivery System", Sakarya University of Applied Sciences, Department of Mechatronics Engineering (team of three; report submitted June 2026).

## Who did what

| Work | Attribution |
|---|---|
| ESP32 C++ firmware (this repository) | Repository author, with AI assistance |
| MATLAB/Simulink modelling and simulation | Repository author |
| Mechanical design (hull, fins, internal layout) | A teammate from the TEKNOFEST 2025 team - **not** the repository author |
| Report, manufacturing, assembly, pool testing | Team work; the per-person split is not documented here |

## Relationship to the rocket firmware

The graduation vehicle ran the **Aug 26 2025** firmware in [`firmware/graduation_aug26_ps5_heading_hold/`](../firmware/graduation_aug26_ps5_heading_hold/). It belongs to the same code lineage as the 2025 competition generation but is **not** the identified competition firmware: it has full PS5 input handling and no mission state machine, and it predates the mission-machine files of Aug 29-31 ([history](DEVELOPMENT_HISTORY.md)).

The report itself describes the hardware, the Simulink model and the pool tests; it does not mention PS5, heading lock or the firmware. The link between this firmware and the pool tests rests on the author's statement together with the source code.

## Hardware (report and firmware)

ESP32 DevKit, ICM-20948 IMU, PCA9685 servo driver with four waterproof servos driving four rear fins, a single rear BLDC thruster with ESC (signal on PCA9685 channel 4), battery pack with distribution board and DC-DC converter, hardware emergency-stop button. The vehicle had **no pressure/depth sensor**.

## Pool tests

Performed in an indoor swimming pool (report section 6). The operator used the PS5 controller for throttle and heading lock; joystick steering was implemented but was not the intended method for the stabilisation tests.

| Test | Method | Report-documented outcome |
|---|---|---|
| Surface and submerged velocity runs | 1 m tape marks, stopwatch | Forward motion achieved at the surface and fully submerged (average speeds in the report) |
| Straight-line run | Camera, path marked on image | Initial divergence while accelerating; no continuing divergence afterwards |
| Heading-correction run | Camera, path marked on image | See below |

**Heading-correction procedure (author-confirmed):** the vehicle was aligned with the desired heading and heading lock was activated with the PS5 controller. After the reference was captured, the orientation was deliberately disturbed, to observe whether the IMU-based controller would correct the heading error through differential fin actuation and return toward the original heading. The report notes that fin authority depends on forward speed.

## Evidence levels

- **Implemented in source:** IMU-based heading-hold loop (yaw error to P-controller to differential top/bottom fins), PS5 lock/throttle interface.
- **Demonstrated qualitatively:** the heading-correction behaviour on hardware, via camera observation reported by the team.
- **Not established:** any quantitative performance. No IMU, fin-command or telemetry logs were recorded. There is no lock-off comparison, no repeatability data and no statement about speeds other than those tested.
- **Not claimed:** autonomous operation, depth control, validated or optimised controller performance, or that the Aug 26 file was the competition firmware.

## Simulation vs. firmware

The Simulink model ([`simulation/graduation_2025/`](../simulation/graduation_2025/)) is a reduced-order pitch/yaw/depth model with discrete PID blocks in radians. The embedded firmware is a separate implementation (degrees, different parameters and limits, ~10 Hz PID update in the reconstructed library, no depth loop). The model supports feasibility reasoning; no document shows its controllers were deployed on the vehicle.

## Media

Test footage and the full report are not included here: footage shows people, and the report cover carries personal identifiers. Media will be added only after review for identifying information and metadata. The simulation figures in this repository are original MATLAB plots.
