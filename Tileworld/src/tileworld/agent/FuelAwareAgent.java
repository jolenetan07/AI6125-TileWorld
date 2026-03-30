package tileworld.agent;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

/**
 * Fuel-aware memory agent.
 *
 * Plans with a hard fuel constraint: only commits to goals that still allow a
 * safe return to a known fuel station with a reserve buffer.
 */
public class FuelAwareAgent extends TWAgent {

    private enum GoalMode {
        TILE,
        HOLE,
        REFUEL,
        EXPLORE
    }

    private static final int MIN_RESERVE_FUEL = 15;
    private static final int SOFT_REFUEL_MARGIN = 8;
    private static final int UNKNOWN_STATION_MIN_FUEL = 80;
    private static final int UNKNOWN_STATION_MAX_TASK_DISTANCE = 6;
    private static final int UNKNOWN_STATION_MAX_EXPLORE_DISTANCE = 5;

    private final String name;
    private transient TWPath currentPath;
    private transient Int2D currentGoal;
    private transient GoalMode currentGoalMode;

    public FuelAwareAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.memory = new MemoryBasedWorkingMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
    }

    @Override
    protected TWThought think() {
        TWEntity objectAtCurrentCell = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());

        if (objectAtCurrentCell instanceof TWFuelStation && getFuelLevel() < Parameters.defaultFuelLevel) {
            clearCurrentPlan();
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        if (objectAtCurrentCell instanceof TWTile && carriedTiles.size() < 3) {
            clearCurrentPlan();
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        if (objectAtCurrentCell instanceof TWHole && hasTile()) {
            clearCurrentPlan();
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        TWEntity fuelStation = selectFuelStation();

        if (mustReturnToFuelStation(fuelStation)) {
            TWDirection stationDirection = getDirectionToEntity(fuelStation, GoalMode.REFUEL);
            if (stationDirection != null) {
                return new TWThought(TWAction.MOVE, stationDirection);
            }
        }

        TWEntity taskTarget = hasTile() ? selectTarget(TWHole.class) : selectTarget(TWTile.class);
        GoalMode taskMode = hasTile() ? GoalMode.HOLE : GoalMode.TILE;

        if (taskTarget != null && isTaskFuelFeasible(taskTarget, fuelStation)) {
            TWDirection nextDirection = getDirectionTowards(taskTarget.getX(), taskTarget.getY(), taskMode);
            if (nextDirection != null) {
                return new TWThought(TWAction.MOVE, nextDirection);
            }
            getMemoryModule().forgetObjectAt(taskTarget.getX(), taskTarget.getY());
        }

        if (fuelStation != null) {
            TWDirection stationDirection = getDirectionToEntity(fuelStation, GoalMode.REFUEL);
            if (stationDirection != null && shouldRefuelSoon(fuelStation)) {
                return new TWThought(TWAction.MOVE, stationDirection);
            }
        }

        Int2D explorationTarget = getMemoryModule().getExplorationTarget();
        if (explorationTarget != null && isExplorationFuelFeasible(explorationTarget, fuelStation)) {
            TWDirection nextDirection = getDirectionTowards(explorationTarget.x, explorationTarget.y, GoalMode.EXPLORE);
            if (nextDirection != null) {
                return new TWThought(TWAction.MOVE, nextDirection);
            }
        }

        // If fuel is low and no safe route is known, conserve fuel by waiting.
        if (fuelStation == null && getFuelLevel() <= getFuelReserve()) {
            clearCurrentPlan();
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }

        clearCurrentPlan();
        return new TWThought(TWAction.MOVE, getRandomDirection());
    }

    @Override
    protected void act(TWThought thought) {
        switch (thought.getAction()) {
            case MOVE:
                try {
                    move(thought.getDirection());
                } catch (CellBlockedException ex) {
                    clearCurrentPlan();
                }
                break;
            case PICKUP:
                TWEntity objectAtCurrentCell = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
                if (objectAtCurrentCell instanceof TWTile) {
                    pickUpTile((TWTile) objectAtCurrentCell);
                    getMemoryModule().forgetObjectAt(getX(), getY());
                    clearCurrentPlan();
                }
                break;
            case PUTDOWN:
                TWEntity holeAtCurrentCell = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
                if (holeAtCurrentCell instanceof TWHole) {
                    putTileInHole((TWHole) holeAtCurrentCell);
                    getMemoryModule().forgetObjectAt(getX(), getY());
                    clearCurrentPlan();
                }
                break;
            case REFUEL:
                refuel();
                clearCurrentPlan();
                break;
        }
    }

    @Override
    public String getName() {
        return name;
    }

    private TWEntity selectTarget(Class<?> targetType) {
        TWEntity visibleTarget = getMemory().getClosestObjectInSensorRange(targetType);
        if (visibleTarget != null) {
            return visibleTarget;
        }

        return getMemoryModule().getClosestRememberedObject(targetType);
    }

    private TWEntity selectFuelStation() {
        TWEntity visibleStation = getMemory().getClosestObjectInSensorRange(TWFuelStation.class);
        if (visibleStation != null) {
            return visibleStation;
        }
        return getMemoryModule().getClosestRememberedObject(TWFuelStation.class);
    }

    private TWDirection getDirectionToEntity(TWEntity target, GoalMode goalMode) {
        if (target == null) {
            return null;
        }
        return getDirectionTowards(target.getX(), target.getY(), goalMode);
    }

    private TWDirection getDirectionTowards(int goalX, int goalY, GoalMode goalMode) {
        if (shouldReplan(goalX, goalY, goalMode)) {
            currentGoal = new Int2D(goalX, goalY);
            currentGoalMode = goalMode;
            currentPath = createPath(getX(), getY(), goalX, goalY);
        }

        if (currentPath == null) {
            return null;
        }

        while (currentPath.hasNext()) {
            TWPathStep step = currentPath.popNext();
            if (step.getDirection() != TWDirection.Z) {
                return step.getDirection();
            }
        }

        clearCurrentPlan();
        return null;
    }

    private boolean shouldReplan(int goalX, int goalY, GoalMode goalMode) {
        return currentGoal == null
                || currentGoal.x != goalX
                || currentGoal.y != goalY
                || currentGoalMode != goalMode
                || currentPath == null
                || !currentPath.hasNext();
    }

    private boolean mustReturnToFuelStation(TWEntity station) {
        if (station == null) {
            return false;
        }
        int stationCost = estimatePathCost(getX(), getY(), station.getX(), station.getY());
        return stationCost != Integer.MAX_VALUE && getFuelLevel() <= stationCost + getFuelReserve();
    }

    private boolean shouldRefuelSoon(TWEntity station) {
        int stationCost = estimatePathCost(getX(), getY(), station.getX(), station.getY());
        if (stationCost == Integer.MAX_VALUE) {
            return false;
        }

        // Soft return trigger: keep a small post-return margin rather than returning too early.
        double fuelAfterReturn = getFuelLevel() - stationCost;
        return fuelAfterReturn < (getFuelReserve() + SOFT_REFUEL_MARGIN);
    }

    private boolean isTaskFuelFeasible(TWEntity target, TWEntity station) {
        if (target == null) {
            return false;
        }

        if (station == null) {
            return isUnknownStationTaskFeasible(target);
        }

        int toTarget = estimatePathCost(getX(), getY(), target.getX(), target.getY());
        int targetToStation = estimatePathCost(target.getX(), target.getY(), station.getX(), station.getY());

        if (toTarget == Integer.MAX_VALUE || targetToStation == Integer.MAX_VALUE) {
            return false;
        }

        int requiredFuel = toTarget + targetToStation + getFuelReserve();
        return getFuelLevel() >= requiredFuel;
    }

    private boolean isExplorationFuelFeasible(Int2D target, TWEntity station) {
        if (target == null) {
            return false;
        }

        if (station == null) {
            int toTarget = estimatePathCost(getX(), getY(), target.x, target.y);
            if (toTarget == Integer.MAX_VALUE) {
                return false;
            }

            return toTarget <= UNKNOWN_STATION_MAX_EXPLORE_DISTANCE
                    && getFuelLevel() >= (toTarget + getFuelReserve())
                    && getFuelLevel() >= UNKNOWN_STATION_MIN_FUEL;
        }

        int toTarget = estimatePathCost(getX(), getY(), target.x, target.y);
        int targetToStation = estimatePathCost(target.x, target.y, station.getX(), station.getY());

        if (toTarget == Integer.MAX_VALUE || targetToStation == Integer.MAX_VALUE) {
            return false;
        }

        int requiredFuel = toTarget + targetToStation + getFuelReserve();
        return getFuelLevel() >= requiredFuel;
    }

    private int estimatePathCost(int startX, int startY, int goalX, int goalY) {
        if (startX == goalX && startY == goalY) {
            return 0;
        }

        TWPath path = createPath(startX, startY, goalX, goalY);
        if (path == null) {
            return Integer.MAX_VALUE;
        }

        int moveCount = 0;
        for (TWPathStep step : path.getpath()) {
            if (step.getDirection() != TWDirection.Z) {
                moveCount++;
            }
        }
        return moveCount;
    }

    private TWPath createPath(int startX, int startY, int goalX, int goalY) {
        int maxSearchDistance = getEnvironment().getxDimension() + getEnvironment().getyDimension();
        AstarPathGenerator astar = new AstarPathGenerator(getEnvironment(), this, maxSearchDistance);
        return astar.findPath(startX, startY, goalX, goalY);
    }

    private int getFuelReserve() {
        return Math.max(MIN_RESERVE_FUEL, Parameters.defaultSensorRange * 4);
    }

    private boolean isUnknownStationTaskFeasible(TWEntity target) {
        int toTarget = estimatePathCost(getX(), getY(), target.getX(), target.getY());
        if (toTarget == Integer.MAX_VALUE) {
            return false;
        }

        return toTarget <= UNKNOWN_STATION_MAX_TASK_DISTANCE
                && getFuelLevel() >= (toTarget + getFuelReserve())
                && getFuelLevel() >= UNKNOWN_STATION_MIN_FUEL;
    }

    private MemoryBasedWorkingMemory getMemoryModule() {
        return (MemoryBasedWorkingMemory) getMemory();
    }

    private void clearCurrentPlan() {
        currentPath = null;
        currentGoal = null;
        currentGoalMode = null;
    }

    private TWDirection getRandomDirection() {
        TWDirection[] directions = new TWDirection[]{TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
        return directions[getEnvironment().random.nextInt(directions.length)];
    }
}


