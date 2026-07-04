# Wall-Aware Rescue-Kit Drop — Design

**Date:** 2026-07-04
**Author:** Florian Wiesner (with Claude)
**Scope:** Technical Challenge end-of-run rescue-kit deployment.

## Problem

At the end of the run the robot returns to the start/exit tile and deploys its
rescue kits (`RunState::END`, `src/main.cpp:565`). The rules require kits to be
dropped **against a wall**. The ejectors fire **sideways** — the left servo
(pin 4) out the robot's left, the right servo (pin 7) out its right — so a kit
only seats against a wall if that wall is on the side the servo faces.

The current call, `ejector.Eject(mapper.GetRescuePacks())`, has two defects:

1. **No wall check** — it hardcodes "left servo first" regardless of where a
   wall actually is, so kits can scatter into open space.
2. **Count truncation** — `Ejector::Eject(ErrorCodes, uint8_t)` runs
   `constrain(amount, 1, 6)`, silently capping an 8-pack drop at 6.

We need a routine that deploys a requested number of kits, choosing servo sides
so every kit lands against a wall, and that correctly handles all 8 packs
(4 per servo) — including the 180° turn needed to bring the far servo's packs
against the same wall.

## Key facts (verified)

- **Single drop point.** `.Eject` has exactly one call site (`main.cpp:576`);
  cameras do **not** fire kits mid-run. All packs come out at the exit tile.
- **A map exists at END**, so `Turn180Degree()` (which wall-aligns and updates
  the map via `EndTurn()`) and `mapper.GetRescuePacks()` are both valid here.
  This is *not* a mapless standalone maneuver.
- **`Turn180Degree()`** (`Driving.cpp:168`) turns to `currentHeading + 180°`.
  Its internal wall-align is a bonus (keeps the robot tight to the wall); the
  map mutation is harmless because the run is already over.
- **Wall detection primitive:** `TofSensors::GetWalls()` returns a bitmask —
  **bit 1 = right, bit 3 = left** — and already filters out ramps. ToF is
  refreshed every loop in `cyclicMainTask`, so the reading is fresh at drop time.
- **Two independent counters.** `GetRescuePacks()` counts victims
  (black = 2, blue = 1) and can exceed 8; the Ejector physically holds ≤ 8
  (4 left + 4 right). The drop count **must be capped at physical remaining**.

## Geometry

The left servo fires left, the right fires right. After an in-place 180° turn,
the servo that was on the wall-facing side is replaced by the opposite servo,
which now faces the **same physical wall**. So the far servo's packs can be
deployed against the same wall by turning 180° first.

## Design

### 1. New dumb primitive on `Ejector`

```cpp
/**
 * @brief  Fires a single servo up to `amount` times without any turning.
 * @param  side   ErrorCodes: left / right — which servo to fire.
 * @param  amount Requested kits to drop from that servo.
 * @return Number of kits actually dropped (limited by that side's remaining).
 */
uint8_t EjectServo(ErrorCodes side, uint8_t amount);
```

- Opens/closes only the named servo `min(amount, thatSide'sRemaining)` times.
- Decrements **only** that side's nibble in `remainingPacks`.
- No `p_driving` use, no turning, no wall logic — all sequencing lives in the
  caller.
- `EjectServo(side, 0)` is a no-op.

The existing `Eject(uint8_t)` and `Eject(ErrorCodes, uint8_t)` methods are
**kept as-is** (unused after this change) to minimize churn.

### 2. New routine in `main.cpp`

```cpp
void dropRescueKits(uint16_t requested);
```

**Logic:**

```
amount = min(requested, GetRemaining(left) + GetRemaining(right));   // cap to physical
if (amount == 0) return;

walls     = tof.GetWalls();
wallLeft  = walls & (1 << 3);
wallRight = walls & (1 << 1);
bothWalls = wallLeft && wallRight;

// Primary servo = the one currently facing a wall.
// Defaults to left when both walls present or none detected.
primary   = (wallRight && !wallLeft) ? right : left;
secondary = other(primary);

dropped   = ejector.EjectServo(primary, amount);
remaining = amount - dropped;

if (remaining > 0 && ejector.GetRemaining(secondary) > 0) {
    if (!bothWalls) robot.Turn180Degree();   // bring secondary servo to the same wall
    ejector.EjectServo(secondary, remaining);
    // No return turn — run is over, heading no longer matters.
}
```

**Case coverage from the single code path:**

| Walls detected | Behavior |
|---|---|
| Left only | Left servo drops against left wall; if more needed, 180° → right servo against same wall. |
| Right only | Right servo primary (symmetric). |
| Both | Left servo → left wall, right servo → right wall, **no turn**. |
| Neither | Fallback: default to left, deploy anyway (kits may not seat against a wall; no warning). |

### 3. Call site

`src/main.cpp:575-576` changes from:

```cpp
uint16_t rescuePacks = mapper.GetRescuePacks();
ejector.Eject(rescuePacks);
```

to:

```cpp
dropRescueKits(mapper.GetRescuePacks());
```

## Decisions

- **No-wall fallback:** deploy on the default (left) side anyway, no UI warning.
- **Old `Eject` methods:** kept in place (not deleted).
- **Return turn:** skipped — robot ends 180° from arrival; run is finished.
- **Turn primitive:** `Turn180Degree()` (consistent with prior code; its
  wall-align helps seat kits; map mutation is harmless at end-of-run).

## Non-goals

- No physical alignment / distance-nudge to the wall before dropping — the
  routine only selects the correct side ("just pick the side").
- No changes to how victims are counted (`GetRescuePacks()` untouched).

## Verification

- `pio run` must build cleanly.
- Full behavior check requires flashing to hardware and observing kits deploy
  against the wall at the exit tile, including an 8-pack drop that exercises the
  180° turn.
