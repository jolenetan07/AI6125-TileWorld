
package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;

public class SimpleTWAgent extends TWAgent {

    private static final int SENSOR_RANGE = 3;
    private static final int TILE_CAPACITY = 3;
    private static final int REFUEL_TARGET = 490;
    private static final int MEMORY_TTL = 24;
    private static final int LOW_FUEL_BUFFER = 22;
    private static final int EXPLORE_FUEL_BUFFER = 28;
    private static final int FUEL_COMMIT_MARGIN = 8;

    private final String name;
    private static boolean spawnedPrinted = false;

    private final AstarPathGenerator pathGenerator;
    private final ArrayList<int[]> patrolPoints = new ArrayList<int[]>();
    private final Map<String, Observation> memoryMap = new HashMap<String, Observation>();

    private int fuelX = -1;
    private int fuelY = -1;
    private int patrolIndex = 0;
    private int patrolStep = 1; // agent1 forward, agent2 backward
    private int internalStep = 0;
    private boolean fuelCommitMode = false;
    private int lastTargetX = -1;
    private int lastTargetY = -1;
    private int moveTargetX = -1;
    private int moveTargetY = -1;

    // Cached visible objects for the current step.
    private TWFuelStation visibleFuel;
    private TWHole visibleHole;
    private TWTile visibleTile;

    private static class Observation {
        String type;
        int x;
        int y;
        int lastSeenStep;

        Observation(String type, int x, int y, int lastSeenStep) {
            this.type = type;
            this.x = x;
            this.y = y;
            this.lastSeenStep = lastSeenStep;
        }
    }

    public SimpleTWAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;

        if (!spawnedPrinted) {
            System.out.println("[SPAWNED] SimpleTWAgent");
            spawnedPrinted = true;
        }

        this.pathGenerator = new AstarPathGenerator(
                this.getEnvironment(),
                this,
                this.getEnvironment().getxDimension() + this.getEnvironment().getyDimension()
        );

        buildGlobalPatrolPoints();
        patrolIndex = getNearestPatrolIndex();
        patrolStep = isConservativeAgent() ? -1 : 1;
    }

    @Override
    public void communicate() {
        // TWEnvironment already schedules sense() before communicate() every step.
        internalStep++;
        scanVisibleArea();
        pruneStaleMemory();

        if (visibleFuel != null) {
            this.getEnvironment().receiveMessage(
                    new Message(this.name, "all", encodeMessage("FUEL", visibleFuel.getX(), visibleFuel.getY(), internalStep))
            );
        }
        if (visibleTile != null) {
            this.getEnvironment().receiveMessage(
                    new Message(this.name, "all", encodeMessage("TILE", visibleTile.getX(), visibleTile.getY(), internalStep))
            );
        }
        if (visibleHole != null) {
            this.getEnvironment().receiveMessage(
                    new Message(this.name, "all", encodeMessage("HOLE", visibleHole.getX(), visibleHole.getY(), internalStep))
            );
        }
    }

    @Override
    protected TWThought think() {
        // sense() and communicate() are already scheduled by TWEnvironment each step.
        moveTargetX = -1;
        moveTargetY = -1;

        readMessages();
        pruneStaleMemory();

        Object realHere = this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());

        if (realHere instanceof TWFuelStation) {
            fuelCommitMode = true;
            if (shouldRefuelOnCurrentCell()) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            fuelCommitMode = false;
        }

        if (mustHeadToFuelNow()) {
            fuelCommitMode = true;
        }

        if (fuelCommitMode && knowsFuel()) {
            if (this.getX() == fuelX && this.getY() == fuelY) {
                if (shouldRefuelOnCurrentCell()) {
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                }
                fuelCommitMode = false;
            } else {
                return planMoveTo(fuelX, fuelY);
            }
        }

        // Before fuel is known, prioritize discovering the fuel station over chasing remembered objects.
        if (!knowsFuel()) {
            int[] patrol = getCurrentPatrolTarget();
            if (patrol[0] == this.getX() && patrol[1] == this.getY()) {
                advancePatrolIndex();
                patrol = getCurrentPatrolTarget();
            }
            return planMoveTo(patrol[0], patrol[1]);
        }

        // Immediate actions.
        if (realHere instanceof TWHole && this.carriedTiles.size() > 0) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (realHere instanceof TWTile && this.carriedTiles.size() < TILE_CAPACITY) {
            // Always exploit a tile we are standing on unless fuel safety is already tight.
            if (!mustHeadToFuelNow() && canAffordStandingPickup()) {
                return new TWThought(TWAction.PICKUP, TWDirection.Z);
            }
        }

        Observation chosenGoal = chooseGoal();
        if (chosenGoal != null) {
            return planMoveTo(chosenGoal.x, chosenGoal.y);
        }

        int[] patrol = getCurrentPatrolTarget();
        if (patrol[0] == this.getX() && patrol[1] == this.getY()) {
            if (knowsFuel() && canSafelyGo(fuelX, fuelY, 0) && this.getFuelLevel() < REFUEL_TARGET - 20) {
                return planMoveTo(fuelX, fuelY);
            }
            return moveThought(getRandomDirection());
        }
        return planMoveTo(patrol[0], patrol[1]);
    }

    @Override
    protected void act(TWThought thought) {
        if (thought == null) {
            return;
        }

        switch (thought.getAction()) {
            case PICKUP: {
                Object realHere = this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                if (realHere instanceof TWTile && this.carriedTiles.size() < TILE_CAPACITY) {
                    TWTile tile = (TWTile) realHere;
                    pickUpTile(tile);
                    forget("TILE", tile.getX(), tile.getY());
                    this.getMemory().removeObject(tile);
                }
                return;
            }
            case PUTDOWN: {
                Object realHere = this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                if (realHere instanceof TWHole && this.carriedTiles.size() > 0) {
                    TWHole hole = (TWHole) realHere;
                    putTileInHole(hole);
                    forget("HOLE", hole.getX(), hole.getY());
                    this.getMemory().removeObject(hole);
                }
                return;
            }
            case REFUEL: {
                if (this.getEnvironment().inFuelStation(this)) {
                    this.refuel();
                }
                return;
            }
            default: {
                if (thought.getDirection() == null || thought.getDirection() == TWDirection.Z) {
                    return;
                }
                if (this.getFuelLevel() <= 0) {
                    return;
                }
                try {
                    this.move(thought.getDirection());
                } catch (CellBlockedException ex) {
                    advancePatrolIndex();
                    if (moveTargetX >= 0 && moveTargetY >= 0) {
                        tryAlternateMoveToward(moveTargetX, moveTargetY);
                    } else {
                        tryAlternateMove();
                    }
                }
            }
        }
    }

    @Override
    public String getName() {
        return name;
    }

    private void readMessages() {
        for (Message m : this.getEnvironment().getMessages()) {
            if (m == null || m.getFrom() == null || m.getMessage() == null) {
                continue;
            }
            if (this.name.equals(m.getFrom())) {
                continue;
            }

            String[] parts = m.getMessage().trim().split(" ");
            if (parts.length != 5 || !"OBS".equals(parts[0])) {
                continue;
            }

            try {
                String type = parts[1];
                int x = Integer.parseInt(parts[2]);
                int y = Integer.parseInt(parts[3]);
                int seenStep = Integer.parseInt(parts[4]);

                if ("FUEL".equals(type) || "TILE".equals(type) || "HOLE".equals(type)) {
                    remember(type, x, y, seenStep);
                    if ("FUEL".equals(type)) {
                        fuelX = x;
                        fuelY = y;
                    }
                }
            } catch (NumberFormatException ignored) {
                // Ignore malformed messages.
            }
        }
    }

    private void scanVisibleArea() {
        visibleFuel = null;
        visibleHole = null;
        visibleTile = null;

        int bestFuelDist = Integer.MAX_VALUE;
        int bestHoleDist = Integer.MAX_VALUE;
        int bestTileDist = Integer.MAX_VALUE;

        int minX = Math.max(0, this.getX() - SENSOR_RANGE);
        int maxX = Math.min(this.getEnvironment().getxDimension() - 1, this.getX() + SENSOR_RANGE);
        int minY = Math.max(0, this.getY() - SENSOR_RANGE);
        int maxY = Math.min(this.getEnvironment().getyDimension() - 1, this.getY() + SENSOR_RANGE);

        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                if (!isInSensorRange(x, y)) {
                    continue;
                }

                Object obj = this.getEnvironment().getObjectGrid().get(x, y);
                int d = Math.abs(this.getX() - x) + Math.abs(this.getY() - y);

                if (!(obj instanceof tileworld.environment.TWObstacle) && this.getMemory().isCellBlocked(x, y)) {
                    this.getMemory().removeAgentPercept(x, y);
                }

                if (obj instanceof TWFuelStation) {
                    if (d < bestFuelDist) {
                        bestFuelDist = d;
                        visibleFuel = (TWFuelStation) obj;
                    }
                    remember("FUEL", x, y);
                    fuelX = x;
                    fuelY = y;
                } else if (obj instanceof TWHole) {
                    if (d < bestHoleDist) {
                        bestHoleDist = d;
                        visibleHole = (TWHole) obj;
                    }
                    remember("HOLE", x, y);
                    forget("TILE", x, y);
                } else if (obj instanceof TWTile) {
                    if (d < bestTileDist) {
                        bestTileDist = d;
                        visibleTile = (TWTile) obj;
                    }
                    remember("TILE", x, y);
                    forget("HOLE", x, y);
                } else {
                    forget("TILE", x, y);
                    forget("HOLE", x, y);
                }
            }
        }
    }

    private void pruneStaleMemory() {
        Iterator<Map.Entry<String, Observation>> it = memoryMap.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, Observation> entry = it.next();
            Observation obs = entry.getValue();
            if ("FUEL".equals(obs.type)) {
                continue;
            }
            if (internalStep - obs.lastSeenStep > MEMORY_TTL) {
                it.remove();
            }
        }
    }

    private void remember(String type, int x, int y) {
        remember(type, x, y, internalStep);
    }

    private void remember(String type, int x, int y, int seenStep) {
        String key = makeKey(type, x, y);
        Observation existing = memoryMap.get(key);
        if (existing == null) {
            memoryMap.put(key, new Observation(type, x, y, seenStep));
        } else {
            existing.lastSeenStep = Math.max(existing.lastSeenStep, seenStep);
        }
    }

    private void forget(String type, int x, int y) {
        memoryMap.remove(makeKey(type, x, y));
    }

    private String makeKey(String type, int x, int y) {
        return type + "@" + x + "," + y;
    }

    private Observation chooseGoal() {
        Observation bestHole = bestReachableObservation("HOLE");
        Observation bestTile = bestReachableObservation("TILE");

        int carried = this.carriedTiles.size();

        if (carried >= TILE_CAPACITY) {
            return bestHole;
        }

        if (carried > 0) {
            if (bestHole != null && bestTile != null) {
                int distHole = pathLength(this.getX(), this.getY(), bestHole.x, bestHole.y);
                int distTile = pathLength(this.getX(), this.getY(), bestTile.x, bestTile.y);
                int tileToHole = pathLength(bestTile.x, bestTile.y, bestHole.x, bestHole.y);

                if (distTile >= 0 && tileToHole >= 0 && canSafelyChainTileThenHole(bestTile, bestHole)) {
                    if (carried == 1 && distTile + tileToHole <= distHole + 3) {
                        return bestTile;
                    }
                    if (carried == 2 && distTile <= 2 && tileToHole <= distHole + 1) {
                        return bestTile;
                    }
                }
                return bestHole;
            }
            if (bestHole != null) {
                return bestHole;
            }
            if (bestTile != null && canSafelyGo(bestTile.x, bestTile.y, exploreReserve())) {
                return bestTile;
            }
            return null;
        }

        return bestTile;
    }

    private Observation bestReachableObservation(String type) {
        Observation best = null;
        int bestScore = Integer.MIN_VALUE;
        int carried = this.carriedTiles.size();

        for (Observation obs : memoryMap.values()) {
            if (!type.equals(obs.type)) {
                continue;
            }

            int manhattan = Math.abs(this.getX() - obs.x) + Math.abs(this.getY() - obs.y);
            int freshness = Math.max(0, MEMORY_TTL - (internalStep - obs.lastSeenStep));
            int score = freshness - (2 * manhattan);

            if ("HOLE".equals(type)) {
                if (carried <= 0) {
                    continue;
                }
                score += 25 + (12 * carried);
                if (visibleHole != null && visibleHole.getX() == obs.x && visibleHole.getY() == obs.y) {
                    score += 20;
                }
            } else {
                if (carried >= TILE_CAPACITY) {
                    continue;
                }
                score += (carried == 0) ? 18 : 8;
                if (visibleTile != null && visibleTile.getX() == obs.x && visibleTile.getY() == obs.y) {
                    score += 16;
                }
                Observation supportingHole = bestSupportingHoleForTile(obs);
                if (supportingHole != null) {
                    int tileToHole = Math.abs(obs.x - supportingHole.x) + Math.abs(obs.y - supportingHole.y);
                    score += Math.max(0, 10 - tileToHole);
                }
            }

            if (score > bestScore) {
                int dist = pathLength(this.getX(), this.getY(), obs.x, obs.y);
                if (dist < 0) {
                    continue;
                }
                if (!isSafeToPursue(obs, dist)) {
                    continue;
                }
                best = obs;
                bestScore = score - dist;
            }
        }

        return best;
    }

    private Observation bestSupportingHoleForTile(Observation tileObs) {
        Observation bestHole = null;
        int bestDist = Integer.MAX_VALUE;
        for (Observation obs : memoryMap.values()) {
            if (!"HOLE".equals(obs.type)) {
                continue;
            }
            int d = Math.abs(tileObs.x - obs.x) + Math.abs(tileObs.y - obs.y);
            if (d < bestDist) {
                bestDist = d;
                bestHole = obs;
            }
        }
        return bestHole;
    }

    private boolean isSafeToPursue(Observation obs, int toGoalDist) {
        if (!knowsFuel()) {
            return this.getFuelLevel() > toGoalDist + exploreReserve();
        }

        int goalToFuel = pathLength(obs.x, obs.y, fuelX, fuelY);
        if (goalToFuel < 0) {
            return false;
        }
        return this.getFuelLevel() > toGoalDist + goalToFuel + refuelReserve();
    }

    private boolean canSafelyChainTileThenHole(Observation tile, Observation hole) {
        int toTile = pathLength(this.getX(), this.getY(), tile.x, tile.y);
        int tileToHole = pathLength(tile.x, tile.y, hole.x, hole.y);
        if (toTile < 0 || tileToHole < 0) {
            return false;
        }

        if (!knowsFuel()) {
            return this.getFuelLevel() > toTile + tileToHole + exploreReserve();
        }

        int holeToFuel = pathLength(hole.x, hole.y, fuelX, fuelY);
        if (holeToFuel < 0) {
            return false;
        }
        return this.getFuelLevel() > toTile + tileToHole + holeToFuel + refuelReserve();
    }

    private boolean canSafelyGo(int tx, int ty, int reserve) {
        TWPath toTarget = pathTo(tx, ty);
        if (toTarget == null || toTarget.getpath() == null) {
            return false;
        }
        int toTargetCost = toTarget.getpath().size();

        if (!knowsFuel()) {
            return this.getFuelLevel() > toTargetCost + Math.max(reserve, exploreReserve());
        }

        TWPath toFuel = pathGenerator.findPath(tx, ty, fuelX, fuelY);
        if (toFuel == null || toFuel.getpath() == null) {
            return false;
        }
        int toFuelCost = toFuel.getpath().size();
        return this.getFuelLevel() > toTargetCost + toFuelCost + Math.max(reserve, refuelReserve());
    }

    private boolean mustHeadToFuelNow() {
        if (!knowsFuel()) {
            return false;
        }
        int toFuel = pathLength(this.getX(), this.getY(), fuelX, fuelY);
        if (toFuel < 0) {
            return false;
        }
        return this.getFuelLevel() <= toFuel + refuelReserve() + (fuelCommitMode ? FUEL_COMMIT_MARGIN : 0);
    }

    private boolean shouldRefuelOnCurrentCell() {
        int target = REFUEL_TARGET;
        if (this.carriedTiles.size() == 1) {
            target += 2;
        } else if (this.carriedTiles.size() >= 2) {
            target += 6;
        }
        if (isConservativeAgent()) {
            target += 3;
        }
        if (fuelCommitMode) {
            target += 6;
        }
        if (target > 500) {
            target = 500;
        }
        return this.getFuelLevel() < target;
    }

    private boolean canAffordStandingPickup() {
        if (!knowsFuel()) {
            return this.getFuelLevel() > exploreReserve();
        }
        int toFuel = pathLength(this.getX(), this.getY(), fuelX, fuelY);
        if (toFuel < 0) {
            return false;
        }
        return this.getFuelLevel() > toFuel + refuelReserve() + 4;
    }

    private int refuelReserve() {
        int reserve = LOW_FUEL_BUFFER;
        if (isConservativeAgent()) {
            reserve += 5;
        }

        int carried = this.carriedTiles.size();
        if (carried == 1) {
            reserve += 6;
        } else if (carried == 2) {
            reserve += 12;
        } else if (carried >= 3) {
            reserve += 18;
        }

        if (fuelCommitMode) {
            reserve += 4;
        }
        return reserve;
    }

    private int exploreReserve() {
        int reserve = EXPLORE_FUEL_BUFFER;
        if (this.carriedTiles.size() > 0) {
            reserve += 6;
        }
        if (isConservativeAgent()) {
            reserve += 4;
        }
        return reserve;
    }

    private boolean isConservativeAgent() {
        return this.name != null && this.name.toLowerCase().endsWith("2");
    }

    private void buildGlobalPatrolPoints() {
        patrolPoints.clear();
        int width = this.getEnvironment().getxDimension();
        int height = this.getEnvironment().getyDimension();

        int startX = (width > 7) ? 3 : 0;
        int endX = (width > 7) ? width - 4 : width - 1;
        int startY = (height > 7) ? 3 : 0;
        int endY = (height > 7) ? height - 4 : height - 1;

        boolean leftToRight = true;
        for (int y = startY; y <= endY; y += 7) {
            if (leftToRight) {
                for (int x = startX; x <= endX; x += 7) {
                    patrolPoints.add(new int[] { clampX(x), clampY(y) });
                }
            } else {
                int lastX = startX;
                for (int x = startX; x <= endX; x += 7) {
                    lastX = x;
                }
                for (int x = lastX; x >= startX; x -= 7) {
                    patrolPoints.add(new int[] { clampX(x), clampY(y) });
                }
            }
            leftToRight = !leftToRight;
        }

        if (patrolPoints.isEmpty()) {
            patrolPoints.add(new int[] { clampX(width / 2), clampY(height / 2) });
        }
    }

    private int[] getCurrentPatrolTarget() {
        if (patrolPoints.isEmpty()) {
            return new int[] { this.getX(), this.getY() };
        }

        for (int tries = 0; tries < patrolPoints.size(); tries++) {
            int[] p = patrolPoints.get(patrolIndex);
            if (this.getX() == p[0] && this.getY() == p[1]) {
                advancePatrolIndex();
                continue;
            }

            TWPath path = pathTo(p[0], p[1]);
            if (path == null || path.getpath() == null || path.getpath().isEmpty()) {
                advancePatrolIndex();
                continue;
            }

            if (!knowsFuel()) {
                if (this.getFuelLevel() > path.getpath().size() + exploreReserve()) {
                    return p;
                }
            } else {
                if (canSafelyGo(p[0], p[1], refuelReserve())) {
                    return p;
                }
            }
            advancePatrolIndex();
        }

        if (knowsFuel()) {
            return new int[] { fuelX, fuelY };
        }
        return new int[] { this.getX(), this.getY() };
    }

    private int getNearestPatrolIndex() {
        if (patrolPoints.isEmpty()) {
            return 0;
        }
        int bestIndex = 0;
        int bestDist = Integer.MAX_VALUE;
        for (int i = 0; i < patrolPoints.size(); i++) {
            int[] p = patrolPoints.get(i);
            int d = Math.abs(this.getX() - p[0]) + Math.abs(this.getY() - p[1]);
            if (d < bestDist) {
                bestDist = d;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    private void advancePatrolIndex() {
        if (patrolPoints.isEmpty()) {
            return;
        }
        patrolIndex += patrolStep;
        if (patrolIndex >= patrolPoints.size()) {
            patrolIndex = 0;
        } else if (patrolIndex < 0) {
            patrolIndex = patrolPoints.size() - 1;
        }
    }

    private TWThought moveThought(TWDirection direction) {
        if (direction == null || direction == TWDirection.Z) {
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }
        return new TWThought(TWAction.MOVE, direction);
    }

    private TWPath pathTo(int tx, int ty) {
        return pathGenerator.findPath(this.getX(), this.getY(), tx, ty);
    }

    private int pathLength(int sx, int sy, int tx, int ty) {
        TWPath path = pathGenerator.findPath(sx, sy, tx, ty);
        if (path == null || path.getpath() == null) {
            return -1;
        }
        return path.getpath().size();
    }

    private TWThought planMoveTo(int tx, int ty) {
        moveTargetX = tx;
        moveTargetY = ty;
        return moveThought(directionTo(tx, ty));
    }

    private TWDirection directionTo(int tx, int ty) {
        TWPath p = pathTo(tx, ty);
        if (p != null && p.getpath() != null && !p.getpath().isEmpty()) {
            return p.getStep(0).getDirection();
        }
        return greedyDirectionTo(tx, ty);
    }

    private TWDirection greedyDirectionTo(int tx, int ty) {
        TWDirection[] dirs = preferredDirectionsTo(tx, ty);
        for (TWDirection dir : dirs) {
            int nx = this.getX() + dir.dx;
            int ny = this.getY() + dir.dy;
            if (!this.getEnvironment().isValidLocation(nx, ny)) {
                continue;
            }
            if (this.getEnvironment().isCellBlocked(nx, ny)) {
                continue;
            }
            return dir;
        }
        return getRandomDirection();
    }

    private void tryAlternateMoveToward(int tx, int ty) {
        TWDirection[] dirs = preferredDirectionsTo(tx, ty);
        for (TWDirection dir : dirs) {
            try {
                if (this.getFuelLevel() > 0) {
                    this.move(dir);
                    return;
                }
            } catch (CellBlockedException ignored) {
                // Try next direction.
            }
        }
        tryAlternateMove();
    }

    private TWDirection[] preferredDirectionsTo(int tx, int ty) {
        int dx = tx - this.getX();
        int dy = ty - this.getY();

        TWDirection primary;
        TWDirection secondary;
        if (Math.abs(dx) >= Math.abs(dy)) {
            primary = (dx >= 0) ? TWDirection.E : TWDirection.W;
            secondary = (dy >= 0) ? TWDirection.S : TWDirection.N;
        } else {
            primary = (dy >= 0) ? TWDirection.S : TWDirection.N;
            secondary = (dx >= 0) ? TWDirection.E : TWDirection.W;
        }

        TWDirection third = opposite(primary);
        TWDirection fourth = opposite(secondary);

        if (primary == secondary) {
            if (primary == TWDirection.E || primary == TWDirection.W) {
                secondary = TWDirection.N;
                fourth = TWDirection.S;
            } else {
                secondary = TWDirection.E;
                fourth = TWDirection.W;
            }
        }

        return new TWDirection[] { primary, secondary, third, fourth };
    }

    private TWDirection opposite(TWDirection dir) {
        if (dir == TWDirection.N) return TWDirection.S;
        if (dir == TWDirection.S) return TWDirection.N;
        if (dir == TWDirection.E) return TWDirection.W;
        return TWDirection.E;
    }

    private void tryAlternateMove() {
        TWDirection[] dirs;
        if (lastTargetX >= 0 && lastTargetY >= 0) {
            dirs = new TWDirection[] {
                    greedyDirectionTo((moveTargetX >= 0 ? moveTargetX : lastTargetX), (moveTargetY >= 0 ? moveTargetY : lastTargetY)),
                    TWDirection.N, TWDirection.E, TWDirection.S, TWDirection.W
            };
        } else {
            dirs = new TWDirection[] { TWDirection.N, TWDirection.E, TWDirection.S, TWDirection.W };
        }
        java.util.HashSet<TWDirection> seen = new java.util.HashSet<TWDirection>();
        for (TWDirection dir : dirs) {
            if (dir == null || dir == TWDirection.Z || seen.contains(dir)) continue;
            seen.add(dir);
            try {
                int nx = this.getX() + dir.dx;
                int ny = this.getY() + dir.dy;
                if (this.getFuelLevel() > 0 && this.getEnvironment().isValidLocation(nx, ny) && !this.getMemory().isCellBlocked(nx, ny)) {
                    this.move(dir);
                    return;
                }
            } catch (CellBlockedException ignored) {
            }
        }
    }

    private TWDirection getRandomDirection() {
        TWDirection[] dirs = new TWDirection[] {
                TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W
        };
        TWDirection dir = dirs[this.getEnvironment().random.nextInt(4)];

        if (this.getX() >= this.getEnvironment().getxDimension() - 1) {
            dir = TWDirection.W;
        } else if (this.getX() <= 0) {
            dir = TWDirection.E;
        } else if (this.getY() >= this.getEnvironment().getyDimension() - 1) {
            dir = TWDirection.N;
        } else if (this.getY() <= 0) {
            dir = TWDirection.S;
        }

        return dir;
    }

    private boolean isInSensorRange(int x, int y) {
        return Math.max(Math.abs(this.getX() - x), Math.abs(this.getY() - y)) <= SENSOR_RANGE;
    }

    private boolean knowsFuel() {
        return fuelX != -1 && fuelY != -1;
    }

    private String encodeMessage(String type, int x, int y, int step) {
        return "OBS " + type + " " + x + " " + y + " " + step;
    }

    private int clampX(int x) {
        if (x < 0) {
            return 0;
        }
        if (x >= this.getEnvironment().getxDimension()) {
            return this.getEnvironment().getxDimension() - 1;
        }
        return x;
    }

    private int clampY(int y) {
        if (y < 0) {
            return 0;
        }
        if (y >= this.getEnvironment().getyDimension()) {
            return this.getEnvironment().getyDimension() - 1;
        }
        return y;
    }
}

