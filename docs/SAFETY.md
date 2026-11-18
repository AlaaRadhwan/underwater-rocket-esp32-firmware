# Known safety limitations

These were found by reading the source. The historical code is preserved as written; **nothing here has been fixed in the repository.** Do not run propulsion hardware from this code without your own safety procedure (props off for bench work, a hardware power cut within reach).

Line numbers refer to [`graduation_aug26_ps5_heading_hold/src/main.cpp`](../firmware/graduation_aug26_ps5_heading_hold/src/main.cpp). The supporting classes are a [reconstruction](../firmware/graduation_aug26_ps5_heading_hold/RECONSTRUCTION.md), so items that depend on them are marked *(library)*.

| # | Limitation | Evidence |
|---|---|---|
| 1 | **The ESC keeps receiving throttle after the PS5 kill button.** Circle only resets the fins once and drops the heading lock; throttle is written to the ESC on every loop iteration regardless of kill state, and the D-pad can still raise it. | `main.cpp:438-444` (kill), `501-502` (unconditional throttle writes) |
| 2 | **No Bluetooth link-loss failsafe.** On disconnect the firmware only restarts the listener. Throttle is unchanged and automatic fin control continues; in manual mode the fins hold their last commanded position. | `main.cpp:504-512` |
| 3 | The Cross button both clears the kill state and is part of the mode-switch chords, so switching mode clears the kill. | `main.cpp:445-456` |
| 4 | A manual-mode sequence (Square) blocks the loop for about 10 s - no IMU update, no kill handling - while driving a fixed 1500 us ESC pulse for 4 s. | `main.cpp:458-467` |
| 5 | If an IMU read fails, the loop returns early: fins and throttle stay as last commanded; there is no neutral/idle action. The recovery routine is blocking. | `main.cpp:515-519, 118-167` |
| 6 | The ESC signal and the IMU share one I2C bus through the PCA9685. If the ESP32 stops updating, the PCA9685 keeps its last PWM registers *(inferred from the device's documented behaviour; not tested)*, so the last ESC pulse could persist. | `main.cpp:96, 502`; inference |
| 7 | Yaw is gyro-integrated with no magnetometer, so the locked heading drifts; the drift was not measured. | `main.cpp:554-560` |
| 8 | Manual stick input has no deadband, so stick drift becomes a fin offset. | `main.cpp:240-242` |
| 9 | AUTO-mode fin control runs at any throttle, including idle on the bench. | `main.cpp:601-606` |
| 10 | *(library)* The preserved controller library prints the yaw error on every update and, because it does not set a PID sample time, `PID_v1` recomputes at its 100 ms default (about 10 Hz) while the main loop targets ~200 Hz. | `lib/Servo_Controller`; `PID_v1` default |
| 11 | Wi-Fi/OTA: the access-point password doubles as the OTA password; there is no signature check or rollback. The 2026 firmware's browser control page is unauthenticated HTTP intended for a closed access point. | `main.cpp:343`; `firmware/2026_wifi_control` |
| 12 | No automated tests and no persistent logging. | repository |

The 2026 experimental firmware adds a 2.5 s control-link timeout that idles the motor; the 2025 mission firmware has a roll-safety supervisor whose call is commented out in the preserved file.

A hardware emergency-stop button appears in the graduation report's bill of materials. What it disconnects is not documented, so it is not treated as a mitigation here.
