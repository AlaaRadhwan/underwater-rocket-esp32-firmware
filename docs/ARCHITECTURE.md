# Architecture (Aug 26 2025 graduation-project variant)

All statements are from the firmware source unless marked otherwise. Line numbers refer to [`firmware/graduation_aug26_ps5_heading_hold/src/main.cpp`](../firmware/graduation_aug26_ps5_heading_hold/src/main.cpp), which differs from the original file only in two sanitised credential lines. The supporting classes are a **reconstruction** (see [RECONSTRUCTION.md](../firmware/graduation_aug26_ps5_heading_hold/RECONSTRUCTION.md)), so gains and library-level behaviour are provisional.

## 1. Sensing-to-actuation signal path

```mermaid
flowchart TB
  subgraph Sense["Sensing (I2C 100 kHz, shared bus)"]
    IMU["ICM-20948<br/>accel + gyro (magnetometer unused)<br/>address 0x69"]
  end
  subgraph Est["Estimation (loop on core 1, ~200 Hz target)"]
    B["bias correction<br/>startup mean + online (ZARU)"]
    MA["8-sample moving average<br/>gyro z only"]
    M["Mahony 6-axis update<br/>Kp 0.5, Ki 0"]
    E["quaternion -> roll / pitch / yaw (deg)"]
  end
  subgraph Ctl["Control"]
    L{"heading locked<br/>and not killed?"}
    P["Servo_Controller::update()<br/>pitch EMA, circular yaw filter<br/>wrapped yaw error -> PID_v1"]
    D["deadband - rate limit<br/>- minimum-step write"]
  end
  IMU --> B --> MA --> M --> E --> L --> P --> D
  D --> PCA["PCA9685 (I2C 0x40, 50 Hz)"]
  PCA --> F0["ch 0  left fin"]
  PCA --> F2["ch 2  right fin"]
  PCA --> F3["ch 3  top fin"]
  PCA --> F1["ch 1  bottom fin"]
```

Every step above is in `main()`'s `loop()` (lines 423-623) except `Servo_Controller`, which is in `lib/`.

## 2. PS5 command map

```mermaid
flowchart LR
  subgraph PS5["DualSense inputs"]
    CIR["Circle"]
    CRO["Cross"]
    R1C["R1 + Cross"]
    L1C["L1 + Cross"]
    TRI["Triangle"]
    SQU["Square (manual mode)"]
    DPD["D-pad up / down"]
    RS["Right stick X / Y"]
  end
  CIR --> K["KILL: fins to 90 deg, heading lock dropped<br/>(lines 438-444)"]
  CRO --> KC["clear kill (445-448)"]
  R1C --> MAN["MANUAL mode (451-453)"]
  L1C --> AUT["AUTO mode (454-456)"]
  TRI --> HL["heading-lock request (470-474, 573-581)"]
  SQU --> SEQ["blocking timed sequence (458-467)"]
  DPD --> TH["manualThrottle +/-100 us, 1000-2000 (485-499)"]
  RS --> MF["manual fins: pitch +/-30 deg, yaw +/-20 deg (477-483)"]
```

Notes: Cross is part of both the kill-clear and the mode chords. Throttle is written to the ESC every loop regardless of kill state (lines 501-502). Automatic heading lock: throttle >= 1200 us for 300 ms captures the current yaw (583-599) and unlocks when throttle falls below 1200 us.

## 3. Actuator mapping and the physical ESC connection

```mermaid
flowchart LR
  ESP["ESP32<br/>SDA 21 / SCL 22"] ==>|"I2C 100 kHz"| BUS(("shared I2C bus"))
  BUS --> IMU["ICM-20948 @0x69"]
  BUS --> PCA["PCA9685 @0x40<br/>50 Hz PWM"]
  PCA -->|"ch 0"| L["left fin servo"]
  PCA -->|"ch 2"| R["right fin servo"]
  PCA -->|"ch 3"| T["top fin servo"]
  PCA -->|"ch 1"| B["bottom fin servo"]
  PCA -->|"ch 4  (physical ESC signal)"| ESC["ESC -> BLDC thruster"]
  ESP -.->|"GPIO 27: LEDC throttle also generated<br/>NOT CONNECTED on the vehicle"| NC["unused"]
```

- The ESC signal wire was connected to **PCA9685 channel 4** (author-confirmed). In code this is `Servo_Module esc(4)` (line 96), written at lines 349, 417, 464-466 and 502.
- `Motor_Module motor(27)` also drives GPIO 27 (lines 95, 348, 501), but nothing was attached there; `motor.armESC()` therefore arms an unused pin, and the real ESC first receives a 1000 us pulse at line 417.
- Mixing: left/right fins are driven oppositely for pitch (`90 + p` / `90 - p`), top/bottom oppositely for yaw. There is no roll control (`manualCombined`'s roll argument is unused).
- Power architecture (report figure, not firmware): pack voltage to the ESC, a 5 V rail to the ESP32 and a regulated servo rail via a DC-DC converter; a hardware emergency-stop button is listed in the report's bill of materials. What it disconnects is not documented.

## 4. Runtime tasks

```mermaid
flowchart LR
  subgraph C0["Core 0 - otaTask (priority 4)"]
    AP["Wi-Fi SoftAP"] --> OTA["ArduinoOTA :3232"]
  end
  subgraph C1["Core 1 - Arduino loop()"]
    PS["PS5 handling"] --> IM["IMU read + estimation"] --> CT["heading lock + AUTO fin update"] --> W["delay(5 ms)"]
  end
  OTA -- "g_otaActive pauses loop;<br/>outputs forced to idle/neutral on OTA start" --> C1
```

Startup order: OTA task, a 5 s window, IMU init (10 retries), 500-sample calibration, servo/ESC init, PS5 begin.

## Implemented parameters

These are the values in the preserved/reconstructed configuration, listed for reference.

| Item | Value | Source |
|---|---|---|
| Mahony | Kp 0.5, Ki 0 | `main.cpp:68-69` |
| IMU I2C | 100 kHz, `Wire` timeout 50 ms | `main.cpp:43, 180` |
| Calibration | 500 samples at 5 ms | `main.cpp:49-50` |
| Online bias (ZARU) | gyro < 0.2 deg/s, |a| within 0.03 g of 1 g, 10 s time constant | `main.cpp:86-91` |
| `dt` clamp | 0.5 to 20 ms | `main.cpp:533` |
| Pitch / yaw controller | Kp 1.5, Ki 0, Kd 0 each (`PID_v1`) | `lib/Servo_Controller` (reconstruction) |
| Controller output limit | +/-25 deg about 90 deg | `lib/Servo_Controller` |
| Filters | pitch EMA 0.10; yaw circular low-pass 0.18 | `lib/Servo_Controller` |
| Deadband | 2 deg pitch, 6 deg yaw | `lib/Servo_Controller` |
| Rate limit | 180 deg/s pitch fins, 120 deg/s yaw fins | `lib/Servo_Controller` |
| PID compute interval | library default (100 ms); the reconstruction does not set it | `PID_v1` default |
| Manual fins | pitch +/-30 deg, yaw +/-20 deg, no stick deadband | `main.cpp:240, 479-480` |
| Throttle | 1000-2000 us, 100 us steps | `main.cpp:485-499` |
| Heading lock (auto) | throttle >= 1200 us for 300 ms | `main.cpp:104-105, 583-599` |
| Fin servo pulse range | 500-2500 us over 0-180 deg | `Servo_Module.h` |

The Simulink model uses different controllers and units; see [simulation/graduation_2025/README.md](../simulation/graduation_2025/README.md).
