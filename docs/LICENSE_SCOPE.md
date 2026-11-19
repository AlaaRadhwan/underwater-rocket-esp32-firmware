# Licence scope

This page records which licence applies to what, what each licence does **not** cover, and how the mapping was checked. It is engineering documentation, not legal advice. The machine-readable mapping is [`REUSE.toml`](../REUSE.toml), the texts are in [`LICENSES/`](../LICENSES/), and [`NOTICE`](../NOTICE) summarises the exclusions.

## 1. Mapping

| Area | Licence | Content |
|---|---|---|
| `firmware/**`, `prototypes/**`, `tools/**`, `.github/**`, repository configuration | **GPL-3.0-or-later** | The author's C++ and web sources (variant `main.cpp` files, `Servo_Controller`, `Servo_Module`, `Motor_Module`, the embedded control page), build configuration, scripts, CI |
| `simulation/**/*.slx`, `simulation/**/*.m` | **MIT** | The author's Simulink model and MATLAB scripts |
| `README.md`, `NOTICE`, `THIRD_PARTY.md`, `docs/**`, all other `*.md`, `simulation/**/figures/**` | **CC-BY-4.0** | Text, diagrams and MATLAB-generated plots |

Copyright holder in every case: 2025-2026 Alaa Radhwan. Markdown files that sit inside code directories (for example the reconstruction notes) are documentation, so CC-BY-4.0 applies to them.

## 2. Why GPL-3.0-or-later for the firmware

- The `mahony6Update` function resembles the Mahony implementation distributed under the x-io statement "GNU General Public Licence unless an alternative licence is provided in source". That statement names **no version**, so any published GPL version may be chosen (GPL-2.0 section 9, GPL-3.0 section 14). A derivation can neither be shown nor excluded ([LICENSING.md](LICENSING.md)); a GPL licence for the firmware is consistent with either answer.
- GPL-3.0 (not 2.0) is compatible with the Apache-2.0 parts of ESP-IDF. The LGPL-2.1 Arduino core permits conversion to GPL (LGPL-2.1 section 3). The MIT/BSD libraries are compatible.
- It lets the historical sources stay **unmodified**: the licence is stated at repository level, not by editing the old files.

## 3. What the licences cover, and what they do not

They cover only copyright in material written by the repository author. The author states the firmware was developed with AI assistance and that no teammate wrote it; how copyright applies to AI-assisted portions varies by jurisdiction and has not been assessed here.

| Material | Status |
|---|---|
| **The `ps5Controller` library** | **Not covered.** Unlicensed, fetched at build time, not redistributed. **Choosing the GPL does not license it or resolve the dependency.** [PS5_DEPENDENCY.md](PS5_DEPENDENCY.md) |
| PID_v1, SparkFun ICM-20948, Adafruit PWM Servo Driver, Adafruit BusIO | Fetched; own MIT/BSD licences ([THIRD_PARTY.md](../THIRD_PARTY.md)) |
| ESP32 Arduino core, ESP-IDF | Fetched; LGPL-2.1 / Apache-2.0 |
| three.js (prototype web viewer) | Loaded from a CDN at run time; MIT; not included |
| Mahony algorithm and the x-io reference code | Published work by others |
| MATLAB and Simulink | Commercial software needed to run the model |
| The vehicle's mechanical design | A teammate's work, not in this repository |
| Report and photographs | Not included; separate decision |
| Pool-test clip (`media/`) | Included: 10 s, no audio, no people, metadata stripped; the author's footage (CC BY 4.0), but the vehicle's design shown in it remains the teammate's |
| Names, marks, the author's identity | No trademark or endorsement rights |

## 4. Consequences

- **Binaries:** firmware built from the PS5-dependent variants contains the unlicensed library, so **compiled binaries must not be distributed**, with or without the GPL. The repository publishes source only, ignores `*.bin`/`*.elf`, and CI uploads no artifacts. `2026_wifi_control` has no PS5 dependency.
- **Combined works:** the GPL governs the author's source files; it cannot make the PS5 library, or a binary containing it, distributable.
- **Incompatible alternatives:** BTstack (non-commercial clause) and the `esp-ps5` licence (personal, non-commercial) are not GPL-compatible for a combined distribution ([LICENSING.md](LICENSING.md)).

## 5. Checks performed

- **REUSE:** `reuse lint` (REUSE tool 6.3.0, specification 3.3) reports the project compliant: every file has copyright and licence information, and only the three licences above are used. The effective licence of individual files was spot-checked (for example code in `firmware/` is GPL, the Markdown beside it is CC-BY, `.slx`/`.m` are MIT, figures are CC-BY).
- **Provenance:** a search of all tracked files found no third-party copyright or licence notices and no vendored third-party code. The one external code reference is the prototype's CDN import of three.js.
- **Compatibility:** no component distributed in this repository carries a licence incompatible with GPL-3.0-or-later. The incompatible items (PS5 library, BTstack, `esp-ps5`) are not distributed here.

## 6. Open items

- No permission from the PS5 library's rights-holders has been requested or obtained.
- The Mahony provenance question stays open; the GPL route is the conservative response, not a finding.
