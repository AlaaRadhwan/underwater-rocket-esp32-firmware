# Development history

The firmware was developed without version control: iterations are dated file snapshots in the author's working folder. This timeline is reconstructed from **file modification dates and a text search for features** - it is *not* a commit history, dates are not authoritative commit times, and file names alone do not say which snapshot ran on hardware.

## Timeline (file dates)

| Snapshot date | Snapshot | Features present (text search) | Role in this repository |
|---|---|---|---|
| 2025-07-19 | `Motor_Module`, early `Servo_Module` | LEDC-based drivers | Library lineage |
| 2025-08-18 | `main_motor` | Mahony, online gyro bias, PS5 manual control, heading lock | Earliest preserved PS5 line |
| 2025-08-20 | `main20_08` | same | |
| 2025-08-24 | `main25_08` | + I2C bus recovery | |
| **2025-08-26** | **`main28_08`** | + OTA task, IMU watchdog; PS5 manual control + heading lock; **no mission state machine** | **[Graduation-project variant](../firmware/graduation_aug26_ps5_heading_hold/)** |
| 2025-08-29 | `main_29_08` | mission state machine added; manual fin control no longer present | Start of the mission line |
| 2025-08-30 | `main_30_08`, `main_after yaw not working`, `main_copy_30_08_not_moving fins` | mission machine; roll-safety code in one copy | Debug iterations (causes not documented) |
| 2025-08-31 | `main_2025`, `winning_main_code_no_roll_safety`, `really winging code`, `Servo_Controller_2025.*` | mission machine, roll-safety code (call commented out in `main_2025`) | **[2025 competition generation](../firmware/2025_teknofest_mission/)** - flashed file not identified |
| 2026-06-02 to 06-04 | `main_2026_no_ps5`, `main_6_2026` | mission variant with web start/stop; stabilise-only variant | Experimental |
| 2026-06-24 | `main.cpp` | stabilise-only, browser control page, command queue, 2.5 s link-loss fail-safe | **[2026 experimental firmware](../firmware/2026_wifi_control/)** - autonomous mission never completed |

## What the snapshots show

- The PS5 manual-control and heading-lock code exists from the earliest preserved snapshot (Aug 18) and is carried through Aug 26. On Aug 29 the mission state machine appears and the manual fin path is no longer present. The Aug 26 file is therefore the last preserved snapshot of the manual-control line.
- The same estimator (hand-written 6-axis Mahony with online gyro bias) and controller wrapper run through all snapshots.
- OTA and I2C recovery were added in the days before Aug 26.

## What is not known

- Which snapshot was flashed for the 2025 competition. Three candidates exist; a flashed binary or serial log would settle it.
- Which supporting-library revision accompanied the Aug 26 snapshot (see [RECONSTRUCTION.md](../firmware/graduation_aug26_ps5_heading_hold/RECONSTRUCTION.md)).
- The cause and fix of the debug iterations ("yaw not working", "not moving fins"): only the file names survive.
- Hardware run records for any snapshot.
