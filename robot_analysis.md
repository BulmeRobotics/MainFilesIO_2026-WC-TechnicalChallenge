# Robot State Machine and Mapping Analysis

## 1. `src/main.cpp` State Machine
The core loop of the robot is handled by `cyclicMainTask()` and `cyclicRunTask()`. When `currentMenuState` is set to `RUN` (via button press), it iterates through the `currentRunState` state machine:
- **`INITIAL`**: Zeros the gyro, resets the mapper, aligns the robot, and prepares the run before moving to `SETTILE`.
- **`SETTILE`**: The robot stops moving. It queries the TOF sensors and color sensor to map the current tile (`mapper.SetTile()`). Moves to `GET_INSTRUCTIONS`.
- **`GET_INSTRUCTIONS`**: Queries the `Mapping` module for the next action:
  - `T_North/East/South/West`: Starts an absolute turn and switches to `TURN`.
  - `D_Forward` / `ramp`: Briefly checks for victims using the color sensor (to drop kits), then sets `DRIVE` to move one tile forward.
  - `MazeFinished`: The mapper believes the robot has returned home.
- **`TURN`**: Executes PID-controlled turning. Can split 180-degree turns into two 90-degree turns to let the cameras stabilize and scan side walls midway. Transitions to `SETTILE`.
- **`DRIVE`**: PID-controlled straight driving. Handles ramps (and spurious ramp aborts). Once a tile is driven, transitions to `SCAN`.
- **`SCAN`**: Updates the mapper's coordinate internally (`mapper.Move()`) and registers ramp geometry before moving to `SETTILE` to repeat the cycle.
- **`END`**: Run completion. Stops motors, ejects rescue packs, plays LED/Buzzer animations, and exits the `RUN` menu state.

## 2. `lib/Mapping` Implementation
`Mapping` uses a graph of `Tile` structs containing 3D coordinates, pointers to neighboring tiles (`north`, `east`, `south`, `west`, `up`, `down`), and traversal weights.
- When `main.cpp` calls `mapper.SetTile()`, the absolute walls are compared, and new placeholder tiles (marked `unexplored`) are spawned for open exits.
- When `main.cpp` calls `mapper.GetInstruction()`, the mapper uses a **Breadth-First Search (BFS)** to locate the closest `unexplored` tile, and then runs **A* Pathfinding** to calculate the optimal path there (applying penalties to ramps or obstacles).
- **Returning Home:** Once BFS can no longer find any `unexplored` tiles, `_RETURN_HOME` is set to true. The mapper sets its target to `0` (the start tile). When the robot arrives at tile 0, `GetInstruction()` returns `Instructionset::MazeFinished`.

## 3. Run End Bug & Fix
I audited the end-of-run logic triggered by `Instructionset::MazeFinished`. There was a critical bug in `main.cpp`:
```cpp
        if(start > other){
          currentRunState = RunState::END;
        }
        // BUG: This always ran, instantly overwriting RunState::END!
        mapper._NOT_HOME = true;
        currentRunState = RunState::GET_INSTRUCTIONS;
```
If the mapper declared the maze finished and the robot verified it was physically on the silver tile, `currentRunState` was briefly set to `END`, but the lines immediately following unconditionally overwrote the state back to `GET_INSTRUCTIONS`. This would cause the robot to continually loop and never drop its rescue kits or properly terminate the run.

**I have patched `src/main.cpp` to correctly encase the retry logic in an `else` block.** Now, if the silver tile is verified, it enters `RunState::END` and properly triggers the shutdown sequence. If there's an odometry/mapping error and it's *not* on the silver tile, it sets `_NOT_HOME = true`, which prompts the mapper to generate a brute-force search path to locate the true silver tile.
