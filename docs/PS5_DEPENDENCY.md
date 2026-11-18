# PS5 controller dependency: provenance, pin and restrictions

The PS5 (DualSense) input in the Aug 26 variant and the 2025 mission firmware uses the third-party Arduino library `ps5Controller`. **That library has no licence.** This page records what it is, exactly what is pinned, and what that means for anyone using or redistributing this repository.

## What it is and who used it

- Lets an ESP32 connect to a DualSense over Bluetooth Classic by presenting itself under the console's Bluetooth address.
- Used by [`firmware/graduation_aug26_ps5_heading_hold`](../firmware/graduation_aug26_ps5_heading_hold/) (full input handling) and [`firmware/2025_teknofest_mission`](../firmware/2025_teknofest_mission/) (initialisation only). `firmware/2026_wifi_control` does not use it.

## Exact pinned version

```
https://github.com/rodneybakiskan/ps5-esp32.git#42b9ca971876ee19fce35e30a6dfc4232e077e65
```

Commit dated 2022-12-16. The pin is verified: the commit's `src/` tree and `library.properties` are **byte-identical** to the copy that was used during development (compared file by file). The library is fetched by PlatformIO at build time; no copy of it is in this repository.

## Provenance

| Step | Repository | Status (checked 2026-11-18) |
|---|---|---|
| PS3 controller library | `jvpernis/esp32-ps3` | Public; no licence file |
| PS4 adaptation | `aed3/PS4-esp32` ("heavily based on" the PS3 work) | Archived 2023-07-12; no licence file |
| PS5 adaptation (the library's author) | `aed3/ps5-esp32` | **Repository no longer exists** |
| Copy used here | `rodneybakiskan/ps5-esp32` | Public; last pushed 2022-12-16; no licence file |

The bundled Bluetooth-stack header files carry Apache-2.0 notices (Broadcom and Google). The rest of the library has no licence statement and no copyright header by its authors. No licence request was found in any of the three repositories' issue trackers.

## Licence status and what it means

- **No licence is granted.** Without one, default copyright applies: no permission to copy, modify or redistribute. Fetching it at build time does not create any such permission.
- **This repository does not contain or redistribute the library**, so publishing the source here does not copy it. Anyone who builds the project downloads it from its own upstream and must make their own determination about use.
- **Do not** vendor, mirror or fork the library into this repository or a derivative of it.
- **Do not distribute compiled firmware** (`.bin`/`.elf`) built from the PS5-dependent variants, because it contains the library. The repository ignores those files and CI does not upload build artifacts.
- **Choosing the GPL for this repository does not change any of this** ([LICENSE_PROPOSAL.md](LICENSE_PROPOSAL.md)).
- **Availability risk:** one upstream copy has already been deleted. If the pinned repository disappears, the PS5-dependent builds stop working; the other variants are unaffected.
- **Reuse:** reuse of the PS5-dependent variants by others is at their own responsibility.

## Alternatives checked

| Option | Licence (from its actual LICENSE file) | Fit |
|---|---|---|
| `HamzaYslmn/esp-ps5` | Custom licence: personal, non-commercial use; attribution and sponsor link required; not OSI-approved. Its README says LGPL-3.0, which conflicts. | Compiles on the pinned toolchain (stub-sketch test only). Different API and no `end()`; needs an adapter and a hardware retest |
| Bluepad32 | Apache-2.0, but its Bluetooth stack (BTstack) restricts use to personal, non-commercial purposes | Needs a different core (custom Arduino board package, or ESP-IDF 5.5+); a port, not a swap |
| Rights-holder permission | n/a | **Not yet requested; no permission has been obtained** |

The working integration is left unchanged. Moving to a licensed implementation, or obtaining permission, would let this dependency be removed from the list of open issues.
