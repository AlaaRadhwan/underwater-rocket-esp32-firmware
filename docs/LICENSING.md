# Licensing status and open questions

**Status: licences applied** (GPL-3.0-or-later for firmware, MIT for simulation, CC BY 4.0 for documentation; see [LICENSE_SCOPE.md](LICENSE_SCOPE.md) and [`REUSE.toml`](../REUSE.toml)). This page records the investigation behind that choice and the questions that remain open. It is engineering analysis, not legal advice.

## 1. Provenance of the `mahony6Update` function

The function (in every firmware variant's `main.cpp`) implements the Mahony complementary filter (Mahony, Hamel, Pflimlin, *IEEE Trans. Automatic Control* 53(5), 2008). The *algorithm and its equations* are published and not protectable. The question is whether the *code* derives from a licensed implementation.

**Compared against:** the `MahonyAHRS.cpp` distributed in the PlatformIO `Mahony` library (PaulStoffregen/MahonyAHRS), whose header reads "Madgwick's implementation of Mahony's AHRS algorithm ... Open-source resources available on this website are provided under the GNU General Public Licence unless an alternative licence is provided in source" (S. Madgwick, x-io, 2011). A secondary 2015 mailing-list report claims the x-io material was at times described as LGPL or CC-BY-SA; the original x-io terms were not re-verified, so treat the upstream licence as **GPL per the file header, with some uncertainty**.

| Aspect | Reference (`updateIMU`) | `mahony6Update` | Reading |
|---|---|---|---|
| Overall structure | normalise accel, estimate gravity from quaternion, error = cross product, optional integral, add to gyro, integrate quaternion, normalise | same sequence | Dictated by the algorithm |
| Cross-product terms | `ay*vz - az*vy`, `az*vx - ax*vz`, `ax*vy - ay*vx` | identical | Standard `a x v` expansion |
| Gain names | `twoKp`, `twoKi` | `twoKp`, `twoKi` | Same naming convention (common in derivatives) |
| Integral block | `if (twoKi > 0) {accumulate} else {zero}` | same branch structure | Same idiom; widely copied |
| Quaternion update | `q0 += (-qb*gx - qc*gy - q3*gz)` etc. with `qa, qb, qc` temporaries | same equations, `qw, qx, qy, qz` temporaries and a separate `halfDt` | Quaternion kinematics; renamed |
| Gravity vector | **half-scaled** (`halfvz = q0*q0 - 0.5 + q3*q3`) | **full** (`2*(...)`, `q0^2 - q1^2 - q2^2 + q3^2`) | Different formulation (effective feedback gain differs by 2x for the same Kp) |
| Zero-accel handling | skips feedback only, still integrates gyro | returns without updating | Different behaviour |
| Time step | member sample frequency | `dt` argument | Different |
| Normalisation | fast inverse square root | `sqrtf` division | Different |
| Comments | block comments in the reference style | different wording | No verbatim comment text |

**Assessment.** There is no evidence of a verbatim copy. The overlap is the algorithm's necessary structure plus a shared naming and idiom lineage (`twoKp`/`twoKi`, integral-or-zero branch) that is common across many descendants of the x-io code; the differences (full-scaled gravity vector, early return, per-call `dt`, no fast inverse sqrt, rewritten comments) indicate it was written or adapted rather than pasted. Because the firmware was developed with AI assistance and this structure is extremely widespread, the function may reflect that lineage without a recorded act of copying. **A derivation from the GPL-noted reference can neither be shown nor excluded**; the residual risk is judged low-to-moderate and is a licensing decision for the author.

**Constraint:** the historical variants must be preserved as written, so rewriting the function would only clean the *current* tree, not the preserved files.

**Options**
1. **Treat the function as potentially GPL-derived** and choose a GPL-compatible licence (e.g. GPL-3.0-or-later) for the firmware directories; add an attribution comment citing Mahony et al. 2008 and the x-io/Madgwick reference. Lowest risk; consistent with preserved history; permissive libraries (MIT/BSD) and LGPL framework code are compatible.
2. **Permissive licence plus a documented rewrite** of the function from the 2008 paper, with a numerical comparison test; historical variants stay with an explicit provenance notice. Higher effort, and the preserved files would still carry the open question.
3. **Per-directory licences** (e.g. permissive for simulation and docs, GPL for firmware). Clear but adds complexity.

**Decision:** the author chose option 1 (GPL-3.0-or-later for the firmware directories), with MIT for the simulation files and CC BY 4.0 for documentation; it is applied in [LICENSE_SCOPE.md](LICENSE_SCOPE.md). This is the conservative response to an open question, not a finding that the function is derived.

## 2. PS5 controller library

The firmware calls `ps5Controller` (`ps5.begin(mac)`, `ps5.Circle()`, `ps5.RStickX()`, ...). Findings:

| Candidate | Licence | Maintained | API compatible with the firmware | Verdict |
|---|---|---|---|---|
| `aed3/ps5-esp32` (original) | none found; repository now returns 404 | no | yes | Unavailable |
| `rodneybakiskan/ps5-esp32` | **no licence** (GitHub reports none); last push 2022-12-16 | no | **yes - `src/` and `library.properties` at commit `42b9ca9` are byte-identical to the copy used during development** | Only drop-in that preserves the exact functionality; **unlicensed** |
| `HamzaYslmn/esp-ps5` | custom source-available licence in `LICENSE` (attribution + sponsor link, personal non-commercial use; "not OSI/FSF approved"); the README says "LGPL-3.0" - **the two conflict**. (A library index also lists it as AGPL-3.0, a third inconsistent reading.) | yes (2026) | **no** - different API (`ps5.cross`, `ps5.rx` members), written for Arduino-ESP32 3.x | Not a drop-in; licence unclear |
| Bluepad32 | Apache-2.0 per its documentation; the ESP32 Bluetooth stack (BTstack) has separate terms (free for open-source use, commercial licence required otherwise) | yes (2026) | no - different pairing model and API | Viable long-term, needs a code port and a different platform/core |
| `esp32-ps3`, `PS4-esp32` (lineage of the original) | no licence found; PS4-esp32 archived | no / archived | n/a | Same unlicensed lineage |

**Why the lineage matters:** the PS5 library descends from `esp32-ps3` and `PS4-esp32`, neither of which carries a licence file. A fork that adds its own licence text cannot grant rights the upstream never granted, so a "licensed fork" would not clear the question by itself.

**Approach used here:** the library is **not vendored**; `platformio.ini` fetches it at build time from a pinned commit. This keeps the real PS5 functionality (the historical source builds unmodified), avoids redistributing unlicensed code, and makes the dependency explicit. If a properly licensed, API-compatible library appears (or Bluepad32 is adopted via a documented port), only the `lib_deps` line and a thin input layer need to change.

## 3. Other components

See [THIRD_PARTY.md](../THIRD_PARTY.md). All other build dependencies are permissively licensed (MIT/BSD), and the ESP32 Arduino core is LGPL/Apache.

## 4. Status of the licensing work

1. Option chosen and applied: `LICENSE`, `LICENSES/`, `REUSE.toml`, `NOTICE`; `reuse lint` passes.
2. No third-party source is redistributed (checked by search of all tracked files).
3. Still open: permission from the PS5 library's rights-holders (not requested), and the Mahony provenance question itself.
