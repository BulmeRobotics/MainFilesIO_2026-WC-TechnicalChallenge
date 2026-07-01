# Ramp Dead-End Recovery — Design

**Date:** 2026-07-01 (Incheon competition)
**Scope:** One bounded addition to the existing ramp state machine (`Driving::RampHandler`) plus one new `Mapping` method.

## Problem

A ramp can lead straight into a wall — it never flattens onto a normal tile. Today, once `_ON_RAMP` is set, `RampHandler` waits for the incline to return to ±4° before ending the ramp. On a dead-end ramp that never happens, so the robot drives into the wall at ramp speed with bumpers disabled and stalls indefinitely.

## Trigger (detection)

Evaluated inside `RampHandler`'s **on-ramp** branch, **after** the existing incline-flat (`±4°`) check, and only while `_RAMP_UP` is set.

Fires when, for **3 consecutive cycles**, ALL of:
- `FRONT` (upper front VL53L4CD) range `< 100 mm`, `GetStatus == VALID`
- All 4 side ToFs (`LEFT_FRONT`, `LEFT_BACK`, `RIGHT_FRONT`, `RIGHT_BACK`) read a wall (`< TOF_SIDE_WALL_THRESHOLD`, i.e. 170 mm), each `GetStatus == VALID`

The 4 side walls confirm the robot sits square/centred on the ramp (not a misaligned false reading). A single non-VALID reading resets the debounce counter to 0.

Ordering rationale: incline-flat is checked first, so a genuinely completed ramp that happens to have a wall on the landing exits via the normal `RAMP_END` path. The dead-end path only fires while still inclined.

## Recovery (Driving)

New private helper `ReverseOffRamp()`:
- **Phase 1** — drive backwards at `RAMP_DEADEND_REV_SPEED` (= `RAMP_SPEED_DOWN`, so it matches the normal ramp-down speed) until incline returns flat (`±4°`), guarded by `RAMP_DEADEND_REV_TIMEOUT` (6 s, generous for a 2-tile ramp) so a stuck gyro cannot reverse the robot out of the maze.
- **Phase 2 (only if flat was actually reached)** — reverse a further fixed `RAMP_DEADEND_CENTER_MM` (150 mm, tune on hardware) onto the already-visited/free pre-ramp tile so the robot ends **centred** on it. `StartAlign` only squares heading, not fore/aft, so this explicit step is what fulfils "center in the tile".
- Then mirror the normal ramp-exit cleanup (run regardless of outcome so bumpers/colour are never left disabled), minus geometry:
  - re-enable bumpers (`EnableBumpers`), `StartAlign()`
  - unfreeze colour sensing (`p_colorSensing->Freeze(false)`)
  - clear the incline sample array (`arrInclineIndex = 0`)
  - `ClearOnRamp()`, `maxRampIncline = 0`, `_RAMP_UP/_RAMP_DOWN/_STAIR = false`
  - **No height change** — the robot returns to the same level it started on.

`ReverseOffRamp` returns `OK` if it reached flat, `TIMEOUT` otherwise. `RampHandler` returns `ErrorCodes::RAMP_DEAD_END` **only** when `ReverseOffRamp` returned `OK` — so the map is never marked black while the robot is still stranded on the ramp.

## Enable / disable

Guarded by a runtime flag `_RAMP_DEADEND_ENABLED` (default `false`), set via `robot.EnableRampDeadEnd(bool)`. main.cpp enables it in `setup()` under `#define RAMP_DEADEND_RECOVERY` (comment out the define to disable the whole feature). Detection is only attempted when the flag is set.

## Mapping — no new method, reuse the black-tile path

At ramp **entry**, `EvaluateRampDecision` already calls `p_mapSys->Move(true)` (Driving.cpp:607), so the mapper is already standing on the ramp tile. `Ramp()` (which creates the z-link) only runs later in SCAN and is never reached on a dead end. `Move()` never changes z — only `Ramp()` does. So the ramp tile is still an ordinary `unexplored` tile at the same z.

Therefore the dead-end is identical to the existing "drove into a black tile" case (`ExecTileBehavior(TileAction::REVERSE)`), minus the forward move which already happened. No `RollbackOne`, no new Mapping method. The `RAMP_DEAD_END` handler in main.cpp does:

```cpp
mapper.SetTile(0x0F, TileType::black);  // mapper already on ramp tile; marks it black (weight 255)
mapper.Move(false);                     // step mapper back to the pre-ramp tile
currentRunState = RunState::SETTILE;
```

`SetTile` on the still-`unexplored` ramp tile cleanly sets `type=black, weight=255`, so A* treats it as impassable and never routes onto the ramp again.

## main.cpp wiring (DRIVE state)

Capture `RampHandler()`'s return once:
- `RAMP_END`  → `SCAN` (unchanged normal path).
- `RAMP_DEAD_END` → `mapper.SetTile(0x0F, black)` + `mapper.Move(false)`, then → `SETTILE` (skips the normal `SCAN` / `mapper.Ramp` geometry path).

## New symbols

- `ErrorCodes::RAMP_DEAD_END` — **appended at the end** of the enum so no persisted/transmitted raw code value shifts (EEPROM stores colour calibration + main speed only, but end-append is free insurance).
- `Driving::DetectRampDeadEnd()` + `ReverseOffRamp()` (private), `deadEndCounter` + `_RAMP_DEADEND_ENABLED` members, `EnableRampDeadEnd(bool)` setter.
- Constants: `RAMP_DEADEND_FRONT_MM = 100`, `RAMP_DEADEND_DEBOUNCE = 3`, `RAMP_DEADEND_REV_SPEED = RAMP_SPEED_DOWN`, `RAMP_DEADEND_REV_TIMEOUT = 6000`, `RAMP_DEADEND_CENTER_MM = 150`.
- `#define RAMP_DEADEND_RECOVERY` toggle in main.cpp.

## Out of scope

- Down-ramp dead-ends (front sensor geometry doesn't support the same trigger; not observed in rescue-maze rules).
- Tuning of the 100 mm / 3-cycle values — verify on hardware in Incheon.
