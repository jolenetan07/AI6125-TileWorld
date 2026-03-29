package tileworld.agent;

import sim.field.grid.ObjectGrid2D;
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

import java.util.ArrayList;
import java.util.List;

/**
 * Utility-based planner agent.
 *
 * Evaluates multiple candidate targets every step, computes a utility score for
 * each feasible option, and executes the action toward the best-scoring target.
 */
public class UtilityBasedPlannerAgent extends TWAgent {

    private enum GoalMode {
        TILE,
        HOLE,
        EXPLORE,
        REFUEL
    }

    private static class Candidate {
        private final GoalMode mode;
        private final Int2D location;
        private final TWPath path;
        private final double utility;

        private Candidate(GoalMode mode, Int2D location, TWPath path, double utility) {
            this.mode = mode;
            this.location = location;
            this.path = path;
            this.utility = utility;
        }
    }

    private static final double DISTANCE_COST = 1.5;
    private static final double HOLE_BASE_UTILITY = 120.0;
    private static final double TILE_BASE_UTILITY = 70.0;
    private static final double EXPLORE_BASE_UTILITY = 18.0;
    private static final double REFUEL_BASE_UTILITY = 90.0;
    private static final int FUEL_SAFETY_MARGIN = 15;
    private static final int FUEL_LOW_THRESHOLD = 80;

    private final String name;
    private transient TWPath currentPath;
    private transient Int2D currentGoal;
    private transient GoalMode currentGoalMode;

    public UtilityBasedPlannerAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
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

        if (objectAtCurrentCell instanceof TWFuelStation && shouldRefuelNow()) {
            clearCurrentPlan();
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        Candidate best = chooseBestCandidate();
        if (best != null) {
            TWDirection direction = getDirectionFromCandidate(best);
            if (direction != null) {
                return new TWThought(TWAction.MOVE, direction);
            }

            if (best.mode == GoalMode.TILE || best.mode == GoalMode.HOLE) {
                getMemoryModule().forgetObjectAt(best.location.x, best.location.y);
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

    private Candidate chooseBestCandidate() {
        List<Candidate> candidates = new ArrayList<Candidate>();

        if (hasTile()) {
            collectCandidates(TWHole.class, GoalMode.HOLE, candidates);
        } else if (carriedTiles.size() < 3) {
            collectCandidates(TWTile.class, GoalMode.TILE, candidates);
        }

        Int2D explorationTarget = getMemoryModule().getExplorationTarget();
        if (explorationTarget != null) {
            addCandidate(GoalMode.EXPLORE, explorationTarget, candidates);
        }

        if (shouldSeekFuel()) {
            collectCandidates(TWFuelStation.class, GoalMode.REFUEL, candidates);
        }

        Candidate best = null;
        for (Candidate candidate : candidates) {
            if (best == null || candidate.utility > best.utility) {
                best = candidate;
            }
        }

        return best;
    }

    private void collectCandidates(Class<?> type, GoalMode mode, List<Candidate> candidates) {
        ObjectGrid2D map = getMemory().getMemoryGrid();
        int width = getEnvironment().getxDimension();
        int height = getEnvironment().getyDimension();

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                Object cell = map.get(x, y);
                if (cell instanceof TWEntity && type.isInstance(cell)) {
                    addCandidate(mode, new Int2D(x, y), candidates);
                }
            }
        }
    }

    private void addCandidate(GoalMode mode, Int2D location, List<Candidate> candidates) {
        TWPath path = createPath(location.x, location.y);
        if (path == null) {
            return;
        }

        int distance = pathDistance(path);
        double utility = computeUtility(mode, distance);
        candidates.add(new Candidate(mode, location, path, utility));
    }

    private double computeUtility(GoalMode mode, int distance) {
        double utility;
        if (mode == GoalMode.HOLE) {
            utility = HOLE_BASE_UTILITY + (10.0 * carriedTiles.size());
        } else if (mode == GoalMode.TILE) {
            utility = TILE_BASE_UTILITY;
        } else if (mode == GoalMode.REFUEL) {
            double fuelNeed = 1.0 - (fuelLevel / Parameters.defaultFuelLevel);
            utility = REFUEL_BASE_UTILITY + (fuelNeed * 120.0);
        } else {
            utility = EXPLORE_BASE_UTILITY;
        }

        utility -= DISTANCE_COST * distance;

        if (mode != GoalMode.REFUEL && fuelLevel - distance < FUEL_SAFETY_MARGIN) {
            utility -= 200.0;
        }

        return utility;
    }

    private TWDirection getDirectionFromCandidate(Candidate best) {
        if (shouldReplan(best)) {
            currentGoal = best.location;
            currentGoalMode = best.mode;
            currentPath = best.path;
        }

        if (currentPath == null) {
            return null;
        }

        while (currentPath.hasNext()) {
            TWPathStep next = currentPath.popNext();
            if (next.getDirection() != TWDirection.Z) {
                return next.getDirection();
            }
        }

        clearCurrentPlan();
        return null;
    }

    private boolean shouldReplan(Candidate best) {
        return currentGoal == null
                || currentGoalMode != best.mode
                || currentGoal.x != best.location.x
                || currentGoal.y != best.location.y
                || currentPath == null
                || !currentPath.hasNext();
    }

    private TWPath createPath(int goalX, int goalY) {
        int maxSearchDistance = getEnvironment().getxDimension() + getEnvironment().getyDimension();
        AstarPathGenerator astar = new AstarPathGenerator(getEnvironment(), this, maxSearchDistance);
        return astar.findPath(getX(), getY(), goalX, goalY);
    }

    private int pathDistance(TWPath path) {
        int steps = path.getpath().size();
        return Math.max(0, steps - 1);
    }

    private MemoryBasedWorkingMemory getMemoryModule() {
        return (MemoryBasedWorkingMemory) getMemory();
    }

    private boolean shouldSeekFuel() {
        return fuelLevel < FUEL_LOW_THRESHOLD;
    }

    private boolean shouldRefuelNow() {
        return fuelLevel < Parameters.defaultFuelLevel;
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

