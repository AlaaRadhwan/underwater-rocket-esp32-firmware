# Licensing proposal (not applied)

**Status: proposal only. No `LICENSE` file has been added and no licence is in force.** This page describes what would be licensed, under what terms, and - just as important - what a licence in this repository can never cover. It is engineering guidance, not legal advice.

## 1. Proposal

| Path | Content | Proposed licence | Reason |
|---|---|---|---|
| `firmware/**`, `prototypes/**` | The author's own C++ and web sources: the variant `main.cpp` files, `Servo_Controller`, `Servo_Module`, `Motor_Module`, the embedded control page | **GPL-3.0-or-later** | Conservative route for the Mahony function (below); lets historical files stay exactly as written |
| `tools/**`, `.github/**` | Scan script, CI workflow | GPL-3.0-or-later (same as the firmware) | Keeps one licence for all code |
| `simulation/**` | The author's `.slx` model and `.m` scripts | **MIT** (recommended; GPL-3.0-or-later is an acceptable alternative for uniformity) | Contains no GPL-derived content; MIT maximises reuse. MATLAB/Simulink themselves are not covered |
| `docs/**`, `README.md`, `figures/` | Text, diagrams, MATLAB-generated plots | **CC BY 4.0** | Prose and original figures; attribution only |
| `media/` (none yet) | Pool footage and stills, if approved | Decide after review (all rights reserved, or CC BY 4.0) | Shows a vehicle whose mechanical design is a teammate's; see section 3 |

## 2. Why GPL-3.0-or-later for the firmware

- The `mahony6Update` function resembles the Mahony implementation distributed under the x-io statement "GNU General Public Licence unless an alternative licence is provided in source". The statement names **no version**, so any published GPL version may be chosen (GPL-2.0 section 9, GPL-3.0 section 14). A derivation can neither be shown nor excluded ([LICENSING.md](LICENSING.md)); licensing the firmware under the GPL is consistent with either answer.
- GPL-3.0 (rather than 2.0) is compatible with the Apache-2.0 parts of ESP-IDF; the LGPL-2.1 Arduino core permits conversion to GPL (LGPL-2.1 section 3); the MIT/BSD libraries are compatible.
- It lets the **historical sources stay unmodified**: the licence is stated at repository level (see section 5), not by editing the old files.

## 3. What the licence would cover

Only the copyright in material **written by the repository author** in the paths above. The author states that the firmware was developed with AI assistance and that no teammate wrote it; how copyright applies to AI-assisted portions varies by jurisdiction and has not been assessed here.

## 4. What it does **not** cover

| Material | Why not | Where it stands |
|---|---|---|
| **The `ps5Controller` library** | Unlicensed third-party code, fetched at build time. **Choosing the GPL does not license it, grant anyone rights to it, or resolve the dependency.** | [PS5_DEPENDENCY.md](PS5_DEPENDENCY.md) |
| PID_v1, SparkFun ICM-20948, Adafruit PWM/BusIO | Fetched libraries under their own MIT/BSD licences | [THIRD_PARTY.md](../THIRD_PARTY.md) |
| ESP32 Arduino core, ESP-IDF | LGPL-2.1 / Apache-2.0, fetched by PlatformIO | |
| The Mahony algorithm and the x-io reference code | Published work by others; the reference code is the reason for the GPL route | [LICENSING.md](LICENSING.md) |
| MATLAB and Simulink | Commercial software needed to run the model | |
| The vehicle's mechanical design | A teammate's work, not in this repository | [GRADUATION_PROJECT.md](GRADUATION_PROJECT.md) |
| Report, photographs, test footage | Not included; any later media needs its own decision | |
| Names, marks, the author's identity | No trademark or endorsement rights | |

## 5. Practical consequences

- **Binaries:** firmware built from the PS5-dependent variants includes the unlicensed library, so **compiled binaries must not be distributed**, with or without the GPL. The repository publishes source only, ignores `*.bin`/`*.elf`, and CI uploads no artifacts.
- **Combined works:** the GPL governs the author's source files; it cannot make the PS5 library, or any binary containing it, distributable.
- **Incompatible alternatives:** BTstack (non-commercial clause) and the `esp-ps5` licence (personal, non-commercial) are not GPL-compatible for a combined distribution ([LICENSING.md](LICENSING.md)).
- **`2026_wifi_control`** has no PS5 dependency, so its binaries do not carry this restriction.

## 6. How to apply it without editing historical files

If the proposal is accepted:

1. Add `LICENSE` (GPL-3.0 text) and `LICENSES/` per the [REUSE](https://reuse.software/) layout.
2. Add a `NOTICE` file summarising section 4.
3. Add `.reuse/dep5` mapping paths to licences, for example:

```
Files: firmware/* prototypes/* tools/* .github/*
Copyright: the repository author
License: GPL-3.0-or-later

Files: simulation/*
Copyright: the repository author
License: MIT

Files: docs/* README.md THIRD_PARTY.md
Copyright: the repository author
License: CC-BY-4.0
```

4. Update the README licence section and `THIRD_PARTY.md`.

No per-file headers are inserted into the historical firmware.

## 7. Decisions needed

- Accept GPL-3.0-or-later for firmware, MIT (or GPL) for simulation, CC BY 4.0 for documentation, or choose otherwise.
- Whether to accept shipping source-only with the documented PS5 restriction, pending permission.
