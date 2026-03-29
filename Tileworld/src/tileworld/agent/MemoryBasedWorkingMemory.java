package tileworld.agent;

import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.TWEntity;
import tileworld.environment.TWObstacle;

/**
 * Working memory for a memory-based agent.
 *
 * Keeps an internal map of the environment by remembering previously observed
 * objects and which cells have already been seen. Unlike the baseline memory,
 * it can retrieve targets outside the current sensor range and provide
 * exploration goals for unseen or stale areas.
 */
public class MemoryBasedWorkingMemory extends TWAgentWorkingMemory {

    private final TWAgent me;
    private final int width;
    private final int height;
    private final TWEntity[][] rememberedObjects;
    private final double[][] lastSeenTimestep;
    private final boolean[][] seenCells;
    private final ObjectGrid2D internalMemoryGrid;

    public MemoryBasedWorkingMemory(TWAgent me, Schedule schedule, int width, int height) {
        super(me, schedule, width, height);
        this.me = me;
        this.width = width;
        this.height = height;
        this.rememberedObjects = new TWEntity[width][height];
        this.lastSeenTimestep = new double[width][height];
        this.seenCells = new boolean[width][height];
        this.internalMemoryGrid = new ObjectGrid2D(width, height);
    }

    @Override
    public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords,
                             Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
        super.updateMemory(sensedObjects, objectXCoords, objectYCoords, sensedAgents, agentXCoords, agentYCoords);

        double now = getCurrentSimulationTime();
        boolean[][] occupiedCellsInView = new boolean[width][height];

        for (int i = 0; i < sensedObjects.size(); i++) {
            Object sensed = sensedObjects.get(i);
            if (!(sensed instanceof TWEntity)) {
                continue;
            }

            TWEntity object = (TWEntity) sensed;
            rememberObject(object, now);
            occupiedCellsInView[object.getX()][object.getY()] = true;
        }

        int sensorRange = Parameters.defaultSensorRange;
        for (int x = me.getX() - sensorRange; x <= me.getX() + sensorRange; x++) {
            for (int y = me.getY() - sensorRange; y <= me.getY() + sensorRange; y++) {
                if (!me.getEnvironment().isInBounds(x, y)) {
                    continue;
                }

                seenCells[x][y] = true;
                lastSeenTimestep[x][y] = now;

                if (!occupiedCellsInView[x][y]) {
                    forgetObjectAt(x, y);
                }
            }
        }
    }

    public TWEntity getClosestRememberedObject(Class<?> type) {
        double now = getCurrentSimulationTime();
        TWEntity best = null;
        double bestDistance = Double.MAX_VALUE;
        double bestTimestamp = -1;

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                TWEntity remembered = rememberedObjects[x][y];
                if (remembered == null) {
                    continue;
                }

                if (isExpired(x, y, now)) {
                    forgetObjectAt(x, y);
                    continue;
                }

                if (!type.isInstance(remembered)) {
                    continue;
                }

                double distance = me.getDistanceTo(x, y);
                if (best == null || distance < bestDistance
                        || (distance == bestDistance && lastSeenTimestep[x][y] > bestTimestamp)) {
                    best = remembered;
                    bestDistance = distance;
                    bestTimestamp = lastSeenTimestep[x][y];
                }
            }
        }

        return best;
    }

    public Int2D getExplorationTarget() {
        double now = getCurrentSimulationTime();
        Int2D bestUnseen = null;
        double bestUnseenDistance = Double.MAX_VALUE;

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                if (seenCells[x][y]) {
                    continue;
                }

                if (isKnownBlocked(x, y, now)) {
                    continue;
                }

                double distance = me.getDistanceTo(x, y);
                if (bestUnseen == null || distance < bestUnseenDistance) {
                    bestUnseen = new Int2D(x, y);
                    bestUnseenDistance = distance;
                }
            }
        }

        if (bestUnseen != null) {
            return bestUnseen;
        }

        Int2D stalestKnownCell = null;
        double oldestTimestamp = Double.MAX_VALUE;
        double bestDistance = Double.MAX_VALUE;

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                if (!seenCells[x][y]) {
                    continue;
                }

                if (isKnownBlocked(x, y, now)) {
                    continue;
                }

                double distance = me.getDistanceTo(x, y);
                double timestamp = lastSeenTimestep[x][y];
                if (stalestKnownCell == null || timestamp < oldestTimestamp
                        || (timestamp == oldestTimestamp && distance < bestDistance)) {
                    stalestKnownCell = new Int2D(x, y);
                    oldestTimestamp = timestamp;
                    bestDistance = distance;
                }
            }
        }

        return stalestKnownCell;
    }

    public void forgetObjectAt(int x, int y) {
        rememberedObjects[x][y] = null;
        internalMemoryGrid.set(x, y, null);
        removeAgentPercept(x, y);
    }

    @Override
    public ObjectGrid2D getMemoryGrid() {
        return internalMemoryGrid;
    }

    private void rememberObject(TWEntity object, double time) {
        int x = object.getX();
        int y = object.getY();
        rememberedObjects[x][y] = object;
        seenCells[x][y] = true;
        lastSeenTimestep[x][y] = time;
        internalMemoryGrid.set(x, y, object);
    }

    private boolean isKnownBlocked(int x, int y, double now) {
        if (isExpired(x, y, now)) {
            forgetObjectAt(x, y);
            return false;
        }

        return rememberedObjects[x][y] instanceof TWObstacle;
    }

    private boolean isExpired(int x, int y, double now) {
        return rememberedObjects[x][y] != null && now - lastSeenTimestep[x][y] > Parameters.lifeTime;
    }

    private double getCurrentSimulationTime() {
        return me.getEnvironment().schedule.getTime();
    }
}

