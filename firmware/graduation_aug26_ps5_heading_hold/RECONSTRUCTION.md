# Aug 26 2025 firmware: what is original and what is reconstructed

**This build is a reconstruction.** The application source is the preserved Aug 26 2025 file. The supporting libraries and build configuration that accompanied it on that date were not preserved, so they were reassembled from the closest surviving snapshots. **Do not assume this build behaves identically to the firmware that ran in the graduation pool tests, or that its dependency versions match the original installation.**

## 1. Components

| Component | Source in this directory | Provenance | Confidence it matches Aug 26 |
|---|---|---|---|
| `src/main.cpp` | Original `main28_08.cpp.txt`, file date 2025-08-26 | Application source as preserved; sanitised (section 3) | **High** (original text; two credential lines replaced) |
| `lib/Servo_Module` | `Servo_Module_2025.*`, file date 2025-08-18 | Only surviving PCA9685 version providing `configureBus()` and the one-argument constructor that `main.cpp` calls | **High** (API-forced; the only candidate) |
| `lib/Motor_Module` | `Motor_Module_2025.*`, file date 2025-07-19 | Only surviving version | **High** |
| `lib/Servo_Controller` | `Servo_Controller_2025.*`, file date 2025-08-31 (`.cpp`), header from 2025-08-18 | Closest *later* snapshot; see section 2 | **Uncertain** - Aug 26 lies between the two surviving snapshots |
| PS5 library | fetched, commit `42b9ca9` (2022-12-16) | `src/` and `library.properties` byte-identical to the copy used during development | **High** for the library code |
| PID, IMU, PCA9685, BusIO, platform, framework | pinned in `platformio.ini` | Versions resolved when this repository's build was verified; the original `.pio` held PID 1.2.1, PCA9685 driver 3.0.2, BusIO 1.17.2 | **Not identical** (newer patch versions pinned) |

## 2. The Servo_Controller choice

Two snapshots of `Servo_Controller.cpp` survive: **2025-08-13** (a backup copy) and **2025-08-31**. Their headers are identical, so both satisfy every call `main.cpp` makes (`begin`, `resetServos`, `initServoAngles`, `setTargetAngles`, `update`, `manualCombined`) and both compile with the rest of the code. The active file on Aug 26 was some state between them and is not recoverable.

| Aspect | Aug 13 snapshot | Aug 31 snapshot (used) |
|---|---|---|
| Proportional gains (pitch / yaw) | 1.0 / 1.0 | 1.5 / 1.5 |
| PID output limit | +/-30 deg | +/-25 deg |
| Yaw input handling | median-of-3 + linear low-pass; **no angle wrapping** | circular (sin/cos) low-pass; **wrapped yaw error** |
| Yaw deadband | declared 6 deg (header) | same |
| Start-up servo sweep (`initServoAngles`) | long multi-step sweep (2000 ms steps) | shorter sweep (500 ms steps) |
| Debug output | "ready" banner | prints yaw error each update |

**Why the Aug 31 snapshot:** it handles the +/-180 degree yaw wrap, which the heading-lock code (`yawRefDeg = yawDeg` on raw +/-180 values) implicitly relies on, and it is the same library state as the 2025 mission firmware in this repository. This is a **judgement, not evidence** that it was installed on Aug 26. If the Aug 13 snapshot was actually active, gains, output limit, yaw handling and the start-up sweep would differ.

## 3. Sanitisation of `src/main.cpp`

The only differences from the original file (line endings normalised to LF; original SHA-256 of the raw bytes: `a8861067879d67bc376a48ce5c465067deb94bfd7ba859722d44152ac20520ec`):

```
36    #define AP_PASS "<value>"                    ->  #include "secrets.h" / #define AP_PASS UWR_AP_PASSWORD
37-39 (one inserted line compensated by deleting a blank line, so all other line numbers are unchanged)
53    static const char* PS5_MAC = "<value>";      ->  static const char* PS5_MAC = UWR_PS5_MAC;
```

The hard-coded Wi-Fi/OTA password and the controller pairing address now come from the git-ignored `include/secrets.h` (copy `secrets.h.example`). Nothing else was changed, including the known issues in [docs/SAFETY.md](../../docs/SAFETY.md). The unmodified original is kept outside this repository.

## 4. Build configuration

`platformio.ini` pins the platform (`espressif32@6.6.0`), each library version, the SparkFun IMU library to a commit hash, and the PS5 library to a commit hash. The original configuration used version ranges and an unpinned git URL.

## 5. Verification

- Compiles with PlatformIO 6.1 for `esp32dev` (flash use ~93.5% of the 1.56 MB OTA partition - a tight margin).
- Not run on hardware from this repository.
- The reconstructed behaviour has **not** been compared against the original installation.
