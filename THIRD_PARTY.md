# Third-party components and provenance

The repository's own licences (GPL-3.0-or-later, MIT, CC BY 4.0; see [NOTICE](NOTICE)) cover only the author's original material and **do not cover anything listed here**.

Nothing below is vendored in this repository: PlatformIO fetches each library at build time from the pinned references in each `platformio.ini`. Licences were read from the downloaded packages unless noted. See [docs/LICENSING.md](docs/LICENSING.md) for the open questions.

| Component | Use | Licence evidence |
|---|---|---|
| PID (`br3ttb/PID`) 1.2.1 - `PID_v1` | PID computation | MIT (stated in `PID_v1.cpp` header) |
| SparkFun ICM-20948 Arduino Library (pinned commit) | IMU driver | MIT for code (`License.md`); hardware files CC BY-SA |
| Adafruit PWM Servo Driver Library 3.0.3 | PCA9685 driver | BSD (`license.txt`) |
| Adafruit BusIO 1.17.4 | I2C helper | MIT (`LICENSE`) |
| ESP32 Arduino core 2.0.14 / ESP-IDF (via `espressif32@6.6.0`) | framework, ArduinoOTA, WebServer, FreeRTOS | LGPL-2.1 / Apache-2.0 (not re-verified here) |
| `ps5Controller` (`rodneybakiskan/ps5-esp32`, pinned commit `42b9ca9`) | PS5 DualSense Bluetooth (2025 mission and Aug 26 variants) | **No licence file.** Descends from `esp32-ps3` and `PS4-esp32`, which also carry none; its bundled Bluetooth-stack headers are Apache-2.0 (Broadcom/Google). **Not redistributed.** See [docs/PS5_DEPENDENCY.md](docs/PS5_DEPENDENCY.md) and docs/LICENSING.md |
| Mahony AHRS algorithm | `mahony6Update` | Algorithm: Mahony, Hamel, Pflimlin, *IEEE TAC* 53(5), 2008. The code is a re-implementation; its resemblance to the GPL-noted x-io/Madgwick `MahonyAHRS` is analysed in docs/LICENSING.md |
| three.js 0.157.0 (early prototype web viewer only) | 3D viewer, loaded from a CDN by `Rocket_Web/app.js` at run time | MIT (by its authors); not included here |
| ESPAsyncWebServer, AsyncTCP, MS5611 (early prototype only) | prototype dependencies | Not fetched or verified here |

## Not included on purpose

Third-party calibration tools and scripts used during development, the unlicensed PS5 library source, vendor product images from the report's wiring figure, test footage and the full graduation report (see [docs/GRADUATION_PROJECT.md](docs/GRADUATION_PROJECT.md)).
