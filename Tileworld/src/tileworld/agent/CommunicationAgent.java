package tileworld.agent;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Communication-focused agent.
 *
 * Broadcasts sensed tile/hole locations each step and uses teammates' reports as
 * fallback targets when local memory has no usable target.
 */
public class CommunicationAgent extends TWAgent {

    private enum GoalMode {
        TILE,
        HOLE,
        SHARED_TILE,
        SHARED_HOLE,
        EXPLORE
    }

    private static final double SHARED_OBSERVATION_TTL = Parameters.lifeTime;

    private final String name;
    private transient TWPath currentPath;
    private transient Int2D currentGoal;
    private transient GoalMode currentGoalMode;

    private final Map<Int2D, Double> sharedTileLocations = new HashMap<Int2D, Double>();
    private final Map<Int2D, Double> sharedHoleLocations = new HashMap<Int2D, Double>();

    public CommunicationAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.memory = new MemoryBasedWorkingMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
    }

    @Override
    public void communicate() {
        List<Int2D> tiles = new ArrayList<Int2D>();
        List<Int2D> holes = new ArrayList<Int2D>();

        int sensorRange = Parameters.defaultSensorRange;
        for (int x = getX() - sensorRange; x <= getX() + sensorRange; x++) {
            for (int y = getY() - sensorRange; y <= getY() + sensorRange; y++) {
                if (!getEnvironment().isInBounds(x, y)) {
                    continue;
                }

                TWEntity observed = (TWEntity) getEnvironment().getObjectGrid().get(x, y);
                if (observed instanceof TWTile) {
                    tiles.add(new Int2D(x, y));
                } else if (observed instanceof TWHole) {
                    holes.add(new Int2D(x, y));
                }
            }
        }

        ObservationMessage message = new ObservationMessage(name, "*", tiles, holes, getEnvironment().schedule.getTime());
        getEnvironment().receiveMessage(message);
    }

    @Override
    protected TWThought think() {
        integrateIncomingMessages();

        TWEntity objectAtCurrentCell = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
        if (objectAtCurrentCell instanceof TWTile && carriedTiles.size() < 3) {
            clearCurrentPlan();
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        if (objectAtCurrentCell instanceof TWHole && hasTile()) {
            clearCurrentPlan();
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        Target target = hasTile() ? selectHoleTarget() : selectTileTarget();
        if (target != null) {
            TWDirection nextDirection = getDirectionTowards(target.location.x, target.location.y, target.goalMode);
            if (nextDirection != null) {
                return new TWThought(TWAction.MOVE, nextDirection);
            }

            if (target.goalMode == GoalMode.SHARED_TILE) {
                sharedTileLocations.remove(target.location);
            } else if (target.goalMode == GoalMode.SHARED_HOLE) {
                sharedHoleLocations.remove(target.location);
            } else {
                getMemoryModule().forgetObjectAt(target.location.x, target.location.y);
            }
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
                    sharedTileLocations.remove(new Int2D(getX(), getY()));
                    clearCurrentPlan();
                }
                break;
            case PUTDOWN:
                TWEntity holeAtCurrentCell = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
                if (holeAtCurrentCell instanceof TWHole) {
                    putTileInHole((TWHole) holeAtCurrentCell);
                    getMemoryModule().forgetObjectAt(getX(), getY());
                    sharedHoleLocations.remove(new Int2D(getX(), getY()));
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

    private void integrateIncomingMessages() {
        double now = getEnvironment().schedule.getTime();
        for (Message message : getEnvironment().getMessages()) {
            if (!(message instanceof ObservationMessage)) {
                continue;
            }
            if (name.equals(message.getFrom())) {
                continue;
            }

            ObservationMessage observationMessage = (ObservationMessage) message;
            for (Int2D tileLocation : observationMessage.getTileLocations()) {
                sharedTileLocations.put(tileLocation, observationMessage.getTimestamp());
            }
            for (Int2D holeLocation : observationMessage.getHoleLocations()) {
                sharedHoleLocations.put(holeLocation, observationMessage.getTimestamp());
            }
        }

        pruneStaleSharedLocations(sharedTileLocations, now);
        pruneStaleSharedLocations(sharedHoleLocations, now);
    }

    private void pruneStaleSharedLocations(Map<Int2D, Double> sharedLocations, double now) {
        List<Int2D> staleKeys = new ArrayList<Int2D>();
        for (Map.Entry<Int2D, Double> entry : sharedLocations.entrySet()) {
            if (now - entry.getValue() > SHARED_OBSERVATION_TTL) {
                staleKeys.add(entry.getKey());
            }
        }
        for (Int2D key : staleKeys) {
            sharedLocations.remove(key);
        }
    }

    private Target selectTileTarget() {
        TWEntity visibleTile = getMemory().getClosestObjectInSensorRange(TWTile.class);
        if (visibleTile != null) {
            return new Target(new Int2D(visibleTile.getX(), visibleTile.getY()), GoalMode.TILE);
        }

        TWEntity rememberedTile = getMemoryModule().getClosestRememberedObject(TWTile.class);
        if (rememberedTile != null) {
            return new Target(new Int2D(rememberedTile.getX(), rememberedTile.getY()), GoalMode.TILE);
        }

        Int2D sharedTile = getClosestSharedLocation(sharedTileLocations);
        if (sharedTile != null) {
            return new Target(sharedTile, GoalMode.SHARED_TILE);
        }

        return null;
    }

    private Target selectHoleTarget() {
        TWEntity visibleHole = getMemory().getClosestObjectInSensorRange(TWHole.class);
        if (visibleHole != null) {
            return new Target(new Int2D(visibleHole.getX(), visibleHole.getY()), GoalMode.HOLE);
        }

        TWEntity rememberedHole = getMemoryModule().getClosestRememberedObject(TWHole.class);
        if (rememberedHole != null) {
            return new Target(new Int2D(rememberedHole.getX(), rememberedHole.getY()), GoalMode.HOLE);
        }

        Int2D sharedHole = getClosestSharedLocation(sharedHoleLocations);
        if (sharedHole != null) {
            return new Target(sharedHole, GoalMode.SHARED_HOLE);
        }

        return null;
    }

    private Int2D getClosestSharedLocation(Map<Int2D, Double> sharedLocations) {
        Int2D bestLocation = null;
        double bestDistance = Double.MAX_VALUE;
        double newestTimestamp = -1;

        for (Map.Entry<Int2D, Double> entry : sharedLocations.entrySet()) {
            Int2D location = entry.getKey();
            double timestamp = entry.getValue();

            double distance = getDistanceTo(location.x, location.y);
            if (bestLocation == null || distance < bestDistance
                    || (distance == bestDistance && timestamp > newestTimestamp)) {
                bestLocation = location;
                bestDistance = distance;
                newestTimestamp = timestamp;
            }
        }

        return bestLocation;
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

    private static class Target {
        private final Int2D location;
        private final GoalMode goalMode;

        private Target(Int2D location, GoalMode goalMode) {
            this.location = location;
            this.goalMode = goalMode;
        }
    }
}

