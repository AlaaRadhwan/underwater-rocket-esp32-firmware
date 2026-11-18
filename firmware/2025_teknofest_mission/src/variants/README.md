# 2025 candidate variants

The 2025 competition used firmware from this generation, but **which file was actually flashed is not yet identified**. Candidates:

| File | Notes |
|---|---|
| `../main.cpp` (from `main_2025.cpp.txt`) | latest by date (2025-08-31); roll-safety call disabled; fin channel 4 collides with ESC channel 4 |
| `really_winging_code.cpp.txt` | near-identical to `main.cpp` (20 differing lines) |
| `winning_main_code_no_roll_safety.cpp.txt` | different fin channel map (0,2,3,1), 15 s legs, 0 s launch delay; the name suggests a competition-era version but this is not confirmed |

Do not cite any one of them as "the competition firmware" until the author confirms it (flashed binary, serial log, or other evidence).
