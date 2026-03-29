# AI6125-TileWorld

### Overview of the Tileworld Codebase

The Tileworld project is a Java-based multi-agent simulation built on the MASON toolkit (a discrete-event simulation library). It models a grid-based environment where agents (robots) must collect tiles (resources) and deposit them into holes to earn points (rewards). The simulation includes obstacles, fuel stations for refueling, and a time limit. Agents can sense their surroundings, plan actions, communicate, and maintain memory. The goal is to implement intelligent agents that maximize collective rewards through coordination and efficient planning.

The codebase is organized into packages under `src/tileworld/`. It includes compiled `.class` files in `bin/` (generated from the source), but you'll work primarily in `src/`. The simulation runs via `TileworldMain.java`, which provides headless (non-GUI) modes for testing and GUI mode via `TWGUI.java`. Agents are instantiated in the environment, and the simulation steps through time, calling each agent's `sense()`, `communicate()`, `think()`, and `act()` methods in sequence.

Below, I'll explain each file/package in detail, based on their purpose, key components, and how they fit into the simulation. I'll also highlight where to find everything and where to edit for the project requirements (implementing planning, memory, communication, and custom agents). Note: Do not modify the `environment/` package or methods like `increaseReward()` in `TWEnvironment.java` (only callable via `putTileInHole()` in agents), as this violates rules and can lead to score penalties.

#### Root-Level Files (`src/tileworld/`)
- **Parameters.java**: Defines global simulation parameters (e.g., grid size `xDimension`/`yDimension`, sensor range, fuel levels, object spawn rates, end time). Used to configure the environment. Edit here if you need to tweak defaults (e.g., for testing), but avoid changing core logic.
- **Parameters2.java**: Appears to be an alternative or extended parameter set (possibly for variations). Similar to `Parameters.java`; check if it's used in your setup.
- **TileworldMain.java** (attached in context): The entry point for running the simulation. Contains multiple `main()` methods:
  - `main(String[] args)`: Runs multiple iterations with random seeds and averages rewards (for benchmarking).
  - `main4()`: Single run with a fixed seed.
  - `main2()`: Handles checkpoints for resuming simulations.
  - `main3()`: Uses MASON's `doLoop()` for advanced headless runs with command-line args (e.g., repeats, seeds, checkpoints).
    - In the Program arguments field, enter the desired args, e.g., -seed 12345 -for 1000 -time 100 (runs with seed 12345 for 1000 steps, printing every 100 steps).
  - This file is for running/testing; don't edit it unless adding custom run modes. Use it to launch simulations and observe agent performance.
- **TWGUI.java**: Provides the graphical interface for the simulation (using MASON's GUIState). Displays the grid, agents, objects, and inspectors. Not for editing; run via this for visual debugging.

#### Agent Package (`src/tileworld/agent/`)
This package handles agent behavior, sensing, memory, communication, and actions. Agents extend `TWAgent` and override key methods.

- **TWAgent.java**: Abstract base class for all agents. Key methods:
  - `sense()`: Uses `TWAgentSensor` to detect nearby objects and update memory.
  - `communicate()`: Sends a `Message` to the environment's broadcast channel (default is empty; override for custom communication).
  - `think()`: Abstract; must be implemented to decide actions based on memory/sensors (returns a `TWThought`).
  - `act(TWThought thought)`: Abstract; executes the thought (e.g., move, pick up tile).
  - Other methods: `move(TWDirection)`, `pickUpTile(TWTile)`, `putTileInHole(TWHole)`, `refuel()` (call these in `act()`). Agents have fuel, score, carried tiles (max 3), sensor, and memory.
  - **Where to edit**: Don't modify this file. Instead, create new agent classes here extending `TWAgent` (e.g., `MyAgent.java`). Override `communicate()`, `think()`, and `act()` to implement your agent's logic. For the project, each team member should create one such class.
- **SimpleTWAgent.java**: A basic example agent that moves randomly and prints its score. Extends `TWAgent`; implements `think()` (random direction) and `act()` (move). Use this as a template for your agents—copy and modify it.
- **TWAgentSensor.java**: Handles sensing: Scans the grid within sensor range and returns objects (tiles, holes, obstacles, agents). Called by `sense()`; updates memory.
- **TWAgentWorkingMemory.java**: Implements a grid-based memory system. Stores sensed objects with timestamps and decay (removes old facts probabilistically). Methods like `getClosestObjectInSensorRange(Class<?>)` help retrieve data. **Where to edit**: Extend this class (e.g., `MyMemory.java`) for custom memory (optional but recommended for better planning). Override methods to add features like probabilistic beliefs or long-term storage.
- **TWAgentPercept.java**: Represents a single sensed fact (e.g., object type, position, time). Used internally by memory.
- **TWThought.java**: Encapsulates an agent's decision: An action (`TWAction`) and optional direction/data. Returned by `think()`.
- **TWAction.java**: Enum for possible actions (e.g., MOVE, PICKUP, PUTDOWN, REFUEL).
- **Message.java**: Basic class for inter-agent communication: Fields for sender (`from`), recipient (`to`), and content (`message`). **Where to edit**: Extend this (e.g., `MyMessage.java`) to encode custom data (e.g., positions, plans). Agents broadcast via `communicate()`; retrieve all messages via `TWEnvironment.getMessages()`.
- **AgentInspector.java**: Provides a GUI inspector for agents (shows details like position, fuel). For visualization.
- **TWAgentPortrayal.java**: Defines how agents appear in the GUI (e.g., color, shape).

#### Environment Package (`src/tileworld/environment/`)
Manages the simulation world. **Do not edit this package**—it's the simulator core. Violating this downgrades scores.

- **TWEnvironment.java**: The core simulation state (extends MASON's `SimState`). Manages grids (`objectGrid` for entities, `agentGrid` for agents), object creation (tiles, holes, obstacles via `TWObjectCreator`), fuel stations, and messages. Key methods:
  - `start()`: Initializes grids and schedules.
  - `step()`: Advances time, spawns/removes objects, calls agent steps.
  - `getMessages()`: Returns all messages from the current step (use in `think()` for communication).
  - `increaseReward()`: Increments global reward (only call indirectly via `putTileInHole()`).
  - Other helpers: `isCellBlocked()`, `canPickupTile()`, etc.
- **TWEntity.java**: Base class for all grid entities (agents, tiles, etc.). Handles position and grid updates.
- **TWTile.java**: Represents collectible tiles (resources).
- **TWHole.java**: Represents holes where tiles are deposited for points.
- **TWObstacle.java**: Static blocks that agents can't pass.
- **TWFuelStation.java**: Fixed location for refueling.
- **TWObject.java**: Base for dynamic objects (tiles, holes, obstacles).
- **TWObjectCreator.java**: Handles spawning objects based on parameters (e.g., mean/deviation for random placement).
- **TWDirection.java**: Enum for directions (N, S, E, W, Z for stay). Used for movement.
- **NeighbourSpiral.java**: Generates a spiral pattern for searching nearby cells (used in memory/sensor).

#### Exceptions Package (`src/tileworld/exceptions/`)
Custom exceptions for errors (e.g., `CellBlockedException` when moving into obstacles, `InsufficientFuelException` when out of fuel). Handle these in `act()` (e.g., catch and replan).

#### Planners Package (`src/tileworld/planners/`)
Handles pathfinding and planning. Critical for high rewards—agents need efficient plans to navigate.

- **TWPlanner.java**: Interface for planners. Methods: `generatePlan()` (creates a path), `hasPlan()`, `voidPlan()`, `getCurrentGoal()`, `execute()` (returns next direction).
- **DefaultTWPlanner.java**: Basic implementation (likely simple greedy or random). Use as a starting point.
- **AstarPathGenerator.java**: Implements A* pathfinding for optimal routes (avoids obstacles, considers fuel). Includes nodes and a sorted list for priority.
- **TWPath.java**: Represents a planned path (list of steps).
- **TWPathGenerator.java**: Generates paths (likely wraps A*).
- **TWPathStep.java**: A single step in a path (position/direction).
- **Where to edit**: Implement your own planner here (e.g., `MyPlanner.java` extending `TWPlanner` or modifying `AstarPathGenerator`). Integrate into agents by instantiating a planner in `think()` and using it to generate actions. A good planner is key for performance—experiment with heuristics, multi-goal planning, or coordination.

### Where to Edit for Project Requirements
- **Planning Module**: Edit/create in `planners/`. Extend `TWPlanner` or modify `AstarPathGenerator` for custom algorithms (e.g., for tile-hole matching). In your agent's `think()`, use the planner to generate paths based on memory.
- **Memory Module (Optional)**: Edit/create in `agent/`. Extend `TWAgentWorkingMemory` (e.g., add maps for beliefs or decay logic). Instantiate your custom memory in the agent's constructor.
- **Communication Module (Optional)**: Edit/create in `agent/. Extend `Message` for structured data (e.g., JSON or custom fields). Override `communicate()` to send info (e.g., agent positions). In `think()`, call `getEnvironment().getMessages()` to read others' broadcasts for coordination.
- **Custom Agents**: Create new files in `agent/` extending `TWAgent` (e.g., `TeamAgent1.java`). Override `communicate()`, `think()`, and `act()`. Use existing methods for actions. Each team member implements one; they run together in the environment.
- **General Tips**: 
  - Test in `TileworldMain.java` (e.g., modify `main()` to instantiate your agents instead of `SimpleTWAgent`).
  - Use `TWEnvironment` methods for queries (e.g., grid access), but don't alter it.
  - For high rewards: Focus on planning (e.g., A* to nearest tile/hole), memory (track object locations), and communication (share goals to avoid conflicts).
  - Compile/run: Use `javac` on `src/` and run from `bin/` or via IDE.
