package tileworld.agent;

import sim.util.Int2D;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

/**
 * Memory-based agent.
 *
 * Uses a persistent internal map of previously observed tiles, holes and
 * obstacles. This allows the agent to move toward remembered targets even when
 * they are no longer within the current sensor range.
 */
public class MemoryBasedAgent extends TWAgent {

    private enum GoalMode {
        TILE,
        HOLE,
        EXPLORE
    }

    private final String name;
    private transient TWPath currentPath;
    private transient Int2D currentGoal;
    private transient GoalMode currentGoalMode;

    public MemoryBasedAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.memory = new MemoryBasedWorkingMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
    }

    @Override
    protected TWThought think() {
        TWEntity objectAtCurrentCell = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());

        if (objectAtCurrentCell instanceof TWTile && carriedTiles.size() < 3) {
            clearCurrentPlan();
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        if (objectAtCurrentCell instanceof TWHole && hasTile()) {
            clearCurrentPlan();
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        TWEntity target = hasTile()
                ? selectTarget(TWHole.class)
                : selectTarget(TWTile.class);

        GoalMode targetMode = hasTile() ? GoalMode.HOLE : GoalMode.TILE;
        if (target != null) {
            TWDirection nextDirection = getDirectionTowards(target.getX(), target.getY(), targetMode);
            if (nextDirection != null) {
                return new TWThought(TWAction.MOVE, nextDirection);
            }

            getMemoryModule().forgetObjectAt(target.getX(), target.getY());
        }

        Int2D explorationTarget = getMemoryModule().getExplorationTarget();
        if (explorationTarget != null) {
            TWDirection nextDirection = getDirectionTowards(explorationTarget.x, explorationTarget.y, GoalMode.EXPLORE);
            if (nextDirection != null) {
                return new TWThought(TWAction.MOVE, nextDirection);
            }
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

    private TWDirection getDirectionTowards(int goalX, int goalY, GoalMode goalMode) {
        if (shouldReplan(goalX, goalY, goalMode)) {
            currentGoal = new Int2D(goalX, goalY);
            currentGoalMode = goalMode;
            currentPath = createPath(goalX, goalY);
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

    private TWPath createPath(int goalX, int goalY) {
        int maxSearchDistance = getEnvironment().getxDimension() + getEnvironment().getyDimension();
        AstarPathGenerator astar = new AstarPathGenerator(getEnvironment(), this, maxSearchDistance);
        return astar.findPath(getX(), getY(), goalX, goalY);
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
        TWDirection[] directions = new TWDirection[]{TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W, TWDirection.Z};
        return directions[getEnvironment().random.nextInt(directions.length)];
    }
}

