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
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Coordinated teamwork agent.
 *
 * This agent is intentionally built on top of the strongest solo strategy in the
 * codebase: utility-based planning. Coordination is added conservatively through
 * shared observations and soft intention/reservation penalties so that teamwork
 * improves coverage without crippling opportunistic local decisions.
 */
public class CoordinatedTeamworkAgent extends TWAgent {

    private enum GoalMode {
        TILE,
        HOLE,
        SHARED_TILE,
        SHARED_HOLE,
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

    private static class ClaimInfo {
        private final String sender;
        private final Int2D senderLocation;
        private final Int2D target;

        private ClaimInfo(String sender, Int2D senderLocation, Int2D target) {
            this.sender = sender;
            this.senderLocation = senderLocation;
            this.target = target;
        }
    }

    private static final double DISTANCE_COST = 1.45;
    private static final double HOLE_BASE_UTILITY = 122.0;
    private static final double TILE_BASE_UTILITY = 72.0;
    private static final double EXPLORE_BASE_UTILITY = 8.0;
    private static final double REFUEL_BASE_UTILITY = 92.0;
    private static final double SHARED_TARGET_BONUS = 0.0;
    private static final double STICKINESS_BONUS = 4.0;
    private static final double NEARBY_TARGET_BONUS = 3.0;
    private static final double CLAIM_STRONG_PENALTY = 24.0;
    private static final double CLAIM_WEAK_PENALTY = 8.0;
    private static final int FUEL_SAFETY_MARGIN = 15;
    private static final int FUEL_LOW_THRESHOLD = 90;
    private static final double SHARED_INFO_TTL = 30.0;
    private static final int NUM_ZONES = 6;
    private static final double ZONE_BONUS = 6.0;
    private static final double LOCAL_TASK_PREFERENCE_MARGIN = 6.0;
    private static final int SHARED_MAX_AGE = 14;
    private static final int SHARED_MAX_DISTANCE = 10;

    private final String name;
    private final int agentIndex;
    private transient TWPath currentPath;
    private transient Int2D currentGoal;
    private transient GoalMode currentGoalMode;

    private final Map<Int2D, Double> sharedTileLocations = new HashMap<Int2D, Double>();
    private final Map<Int2D, Double> sharedHoleLocations = new HashMap<Int2D, Double>();
    private final Map<Int2D, ClaimInfo> tileClaims = new HashMap<Int2D, ClaimInfo>();
    private final Map<Int2D, ClaimInfo> holeClaims = new HashMap<Int2D, ClaimInfo>();
    private transient Int2D sharedFuelStationLocation;
    private transient double sharedFuelStationTimestamp = -1;
    /** Permanent store for fuel-station location — fuel stations never disappear so we must never forget them. */
    private transient Int2D permanentFuelStationLocation;

    public CoordinatedTeamworkAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.agentIndex = parseAgentIndex(name);
        this.memory = new MemoryBasedWorkingMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
    }

    private static int parseAgentIndex(String name) {
        try {
            String digits = name.replaceAll("[^0-9]", "");
            if (!digits.isEmpty()) {
                return Math.max(0, Math.min(NUM_ZONES - 1, Integer.parseInt(digits) - 1));
            }
        } catch (NumberFormatException ignored) {}
        return 0;
    }

    @Override
    public void communicate() {
        List<Int2D> tiles = new ArrayList<>();
        List<Int2D> holes = new ArrayList<>();

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
                } else if (observed instanceof TWFuelStation) {
                    // Update permanent memory whenever station is directly visible.
                    permanentFuelStationLocation = new Int2D(x, y);
                }
            }
        }

        // Always broadcast the permanently-known station location so teammates
        // never forget it even when they haven't been near the station recently.
        Int2D stationToBroadcast = permanentFuelStationLocation;

        Int2D claimedTarget = currentGoal == null ? null : new Int2D(currentGoal.x, currentGoal.y);
        String claimedGoal = getClaimableGoalName(currentGoalMode);

        CoordinationMessage message = new CoordinationMessage(
                name,
                "*",
                tiles,
                holes,
                getEnvironment().schedule.getTime(),
                new Int2D(getX(), getY()),
                claimedTarget,
                claimedGoal,
                stationToBroadcast
        );
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
            } else if (best.mode == GoalMode.SHARED_TILE) {
                sharedTileLocations.remove(best.location);
            } else if (best.mode == GoalMode.SHARED_HOLE) {
                sharedHoleLocations.remove(best.location);
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

    private Candidate chooseBestCandidate() {
        List<Candidate> candidates = new ArrayList<>();

        // Deliver-first policy: if carrying any tile, prioritize finding a hole.
        if (hasTile()) {
            collectLocalCandidates(TWHole.class, GoalMode.HOLE, candidates);
            if (candidates.isEmpty()) {
                collectSharedCandidatesExcluding(GoalMode.SHARED_HOLE, sharedHoleLocations, candidates);
            }
        } else if (carriedTiles.size() < 3) {
            collectLocalCandidates(TWTile.class, GoalMode.TILE, candidates);
            if (candidates.isEmpty()) {
                collectSharedCandidatesExcluding(GoalMode.SHARED_TILE, sharedTileLocations, candidates);
            }
        }

        if (shouldSeekFuel()) {
            Int2D station = getKnownFuelStationLocation();
            if (station != null) {
                addCandidate(GoalMode.REFUEL, station, candidates);
            }
        }

        // Exploration is fallback behavior: avoid stealing cycles from known tasks.
        if (!hasTaskCandidate(candidates)) {
            Int2D zoneTarget = getMemoryModule().getExplorationTargetInRegion(getZoneMinX(), getZoneMaxX());
            if (zoneTarget != null) {
                addCandidate(GoalMode.EXPLORE, zoneTarget, candidates);
            }

            Int2D globalTarget = getMemoryModule().getExplorationTarget();
            if (globalTarget != null && (zoneTarget == null || !zoneTarget.equals(globalTarget))) {
                addCandidate(GoalMode.EXPLORE, globalTarget, candidates);
            }
        }

        Candidate best = null;
        Candidate bestLocal = null;
        for (Candidate candidate : candidates) {
            if (best == null || candidate.utility > best.utility) {
                best = candidate;
            }

            if (candidate.mode == GoalMode.TILE || candidate.mode == GoalMode.HOLE) {
                if (bestLocal == null || candidate.utility > bestLocal.utility) {
                    bestLocal = candidate;
                }
            }
        }

        // Prefer strong local task options over similar-scoring shared goals.
        if (best != null
                && (best.mode == GoalMode.SHARED_TILE || best.mode == GoalMode.SHARED_HOLE)
                && bestLocal != null
                && pathDistance(bestLocal.path) <= Parameters.defaultSensorRange + 2
                && bestLocal.utility + LOCAL_TASK_PREFERENCE_MARGIN >= best.utility) {
            return bestLocal;
        }
        return best;
    }

    private boolean hasTaskCandidate(List<Candidate> candidates) {
        for (Candidate candidate : candidates) {
            if (candidate.mode == GoalMode.TILE
                    || candidate.mode == GoalMode.HOLE
                    || candidate.mode == GoalMode.SHARED_TILE
                    || candidate.mode == GoalMode.SHARED_HOLE) {
                return true;
            }
        }
        return false;
    }

    /**
     * Adds shared candidates while skipping any location already covered by a
     * local candidate, avoiding duplicate evaluations for the same cell.
     */
    private void collectSharedCandidatesExcluding(GoalMode sharedMode,
                                                   Map<Int2D, Double> sharedLocations,
                                                   List<Candidate> candidates) {
        Set<Int2D> covered = new HashSet<>();
        for (Candidate c : candidates) {
            covered.add(c.location);
        }
        for (Map.Entry<Int2D, Double> entry : sharedLocations.entrySet()) {
            Int2D sharedLocation = entry.getKey();
            if (!covered.contains(sharedLocation) && isSharedUseful(sharedLocation, entry.getValue())) {
                addCandidate(sharedMode, sharedLocation, candidates);
            }
        }
    }

    private boolean isSharedUseful(Int2D location, double timestamp) {
        double age = getEnvironment().schedule.getTime() - timestamp;
        if (age > SHARED_MAX_AGE) {
            return false;
        }

        int manhattanDistance = Math.abs(getX() - location.x) + Math.abs(getY() - location.y);
        return manhattanDistance <= SHARED_MAX_DISTANCE;
    }

    /**
     * Returns an exploration target preferring unseen cells in this agent's
     * assigned zone first, falling back to global exploration.
     */
    private Int2D getZoneAwareExplorationTarget() {
        Int2D zoneTarget = getMemoryModule().getExplorationTargetInRegion(getZoneMinX(), getZoneMaxX());
        if (zoneTarget != null) {
            return zoneTarget;
        }
        return getMemoryModule().getExplorationTarget();
    }

    private int getZoneMinX() {
        int zoneWidth = getEnvironment().getxDimension() / NUM_ZONES;
        return agentIndex * zoneWidth;
    }

    private int getZoneMaxX() {
        int envWidth = getEnvironment().getxDimension();
        int zoneWidth = envWidth / NUM_ZONES;
        return (agentIndex == NUM_ZONES - 1) ? envWidth - 1 : (agentIndex + 1) * zoneWidth - 1;
    }

    private boolean isInMyZone(Int2D location) {
        return location.x >= getZoneMinX() && location.x <= getZoneMaxX();
    }

    private void collectLocalCandidates(Class<?> type,
                                   GoalMode localMode,
                                   List<Candidate> candidates) {
        ObjectGrid2D map = getMemory().getMemoryGrid();
        int width = getEnvironment().getxDimension();
        int height = getEnvironment().getyDimension();

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                Object cell = map.get(x, y);
                if (cell instanceof TWEntity && type.isInstance(cell)) {
                    addCandidate(localMode, new Int2D(x, y), candidates);
                }
            }
        }
    }

    private void collectSharedCandidates(GoalMode sharedMode,
                                         Map<Int2D, Double> sharedLocations,
                                         List<Candidate> candidates) {
        for (Int2D sharedLocation : sharedLocations.keySet()) {
            addCandidate(sharedMode, sharedLocation, candidates);
        }
    }

    private void addCandidate(GoalMode mode, Int2D location, List<Candidate> candidates) {
        TWPath path = createPath(location.x, location.y);
        if (path == null) {
            return;
        }

        int distance = pathDistance(path);
        if (!isFuelFeasible(mode, location, distance)) {
            return;
        }

        double utility = computeUtility(mode, location, distance);
        candidates.add(new Candidate(mode, location, path, utility));
    }

    private boolean isFuelFeasible(GoalMode mode, Int2D location, int distanceToTarget) {
        if (mode == GoalMode.REFUEL) {
            return true;
        }

        Int2D station = getKnownFuelStationLocation();
        if (station == null) {
            return true;
        }

        // Use Manhattan lower bound: if this already fails, real path cannot succeed.
        int targetToStationLowerBound = Math.abs(location.x - station.x) + Math.abs(location.y - station.y);
        return fuelLevel >= distanceToTarget + targetToStationLowerBound + FUEL_SAFETY_MARGIN;
    }

    private double computeUtility(GoalMode mode, Int2D location, int distance) {
        double utility;
        if (mode == GoalMode.HOLE || mode == GoalMode.SHARED_HOLE) {
            utility = HOLE_BASE_UTILITY + (10.0 * carriedTiles.size());
        } else if (mode == GoalMode.TILE || mode == GoalMode.SHARED_TILE) {
            utility = TILE_BASE_UTILITY;
        } else if (mode == GoalMode.REFUEL) {
            double fuelNeed = 1.0 - (fuelLevel / Parameters.defaultFuelLevel);
            utility = REFUEL_BASE_UTILITY + (fuelNeed * 120.0);
        } else {
            utility = EXPLORE_BASE_UTILITY;
        }

        if (mode == GoalMode.SHARED_TILE || mode == GoalMode.SHARED_HOLE) {
            utility += SHARED_TARGET_BONUS;
            utility -= computeSharedAgePenalty(mode, location);
        }

        // Spatial task allocation: prefer targets in this agent's assigned zone.
        // ZONE_BONUS / DISTANCE_COST ≈ 15 steps — agents will travel up to ~15
        // extra steps to stay in their zone rather than competing with neighbours.
        if (mode == GoalMode.EXPLORE && isInMyZone(location)) {
            utility += ZONE_BONUS;
        }

        utility -= DISTANCE_COST * distance;
        utility -= computeClaimPenalty(mode, location, distance);
        utility += computeFuelUtilityAdjustment(mode, distance);

        if (mode != GoalMode.EXPLORE && distance <= Parameters.defaultSensorRange + 1) {
            utility += NEARBY_TARGET_BONUS;
        }

        if (currentGoal != null && currentGoal.equals(location) && isSameTaskFamily(currentGoalMode, mode)) {
            utility += STICKINESS_BONUS;
        }

        return utility;
    }

    private double computeFuelUtilityAdjustment(GoalMode mode, int distance) {
        if (mode == GoalMode.REFUEL) {
            return 0.0;
        }

        if (fuelLevel - distance < FUEL_SAFETY_MARGIN) {
            return -200.0;
        }
        return 0.0;
    }

    private double computeClaimPenalty(GoalMode mode, Int2D location, int myDistance) {
        if (mode == GoalMode.EXPLORE || mode == GoalMode.REFUEL) {
            return 0.0;
        }

        if (myDistance <= Parameters.defaultSensorRange) {
            return 0.0;
        }

        // Penalty applies to ALL task modes (TILE, HOLE, SHARED_TILE, SHARED_HOLE).
        // Without this, all agents pick the same locally-remembered tile simultaneously
        // because the fallback "if TILE/HOLE return 0" meant no penalty was ever paid.
        ClaimInfo claim = getRelevantClaim(mode, location);
        if (claim == null || name.equals(claim.sender)) {
            return 0.0;
        }

        int teammateDistance = estimateClaimDistance(claim);
        if (teammateDistance == Integer.MAX_VALUE) {
            return CLAIM_WEAK_PENALTY;
        }

        if (teammateDistance + 2 < myDistance) {
            return CLAIM_STRONG_PENALTY;
        }
        if (teammateDistance <= myDistance) {
            return CLAIM_WEAK_PENALTY;
        }
        return 0.0;
    }

    private double computeSharedAgePenalty(GoalMode mode, Int2D location) {
        Double timestamp = null;
        if (mode == GoalMode.SHARED_TILE) {
            timestamp = sharedTileLocations.get(location);
        } else if (mode == GoalMode.SHARED_HOLE) {
            timestamp = sharedHoleLocations.get(location);
        }

        if (timestamp == null) {
            return 0.0;
        }

        double age = getEnvironment().schedule.getTime() - timestamp;
        return Math.min(24.0, age * 0.8);
    }

    private ClaimInfo getRelevantClaim(GoalMode mode, Int2D location) {
        if (mode == GoalMode.TILE || mode == GoalMode.SHARED_TILE) {
            return tileClaims.get(location);
        }
        if (mode == GoalMode.HOLE || mode == GoalMode.SHARED_HOLE) {
            return holeClaims.get(location);
        }
        return null;
    }

    private void integrateIncomingMessages() {
        double now = getEnvironment().schedule.getTime();
        tileClaims.clear();
        holeClaims.clear();

        for (Message message : getEnvironment().getMessages()) {
            if (isSelfMessage(message)) {
                continue;
            }

            if (message instanceof ObservationMessage) {
                ObservationMessage observationMessage = (ObservationMessage) message;
                for (Int2D tileLocation : observationMessage.getTileLocations()) {
                    sharedTileLocations.put(tileLocation, observationMessage.getTimestamp());
                }
                for (Int2D holeLocation : observationMessage.getHoleLocations()) {
                    sharedHoleLocations.put(holeLocation, observationMessage.getTimestamp());
                }
            }

            if (message instanceof CoordinationMessage) {
                CoordinationMessage coordinationMessage = (CoordinationMessage) message;
                rememberFuelStation(coordinationMessage.getFuelStationLocation(), coordinationMessage.getTimestamp());
                registerClaim(coordinationMessage);
            }
        }

        pruneStale(sharedTileLocations, now);
        pruneStale(sharedHoleLocations, now);
        if (sharedFuelStationLocation != null && now - sharedFuelStationTimestamp > SHARED_INFO_TTL) {
            sharedFuelStationLocation = null;
            sharedFuelStationTimestamp = -1;
        }
    }

    private void registerClaim(CoordinationMessage message) {
        Int2D target = message.getIntendedTarget();
        String goalName = message.getIntendedGoal();
        if (target == null || goalName == null) {
            return;
        }

        ClaimInfo claim = new ClaimInfo(message.getFrom(), message.getSenderLocation(), target);
        if ("TILE".equals(goalName) || "SHARED_TILE".equals(goalName)) {
            storeBestClaim(tileClaims, claim);
        } else if ("HOLE".equals(goalName) || "SHARED_HOLE".equals(goalName)) {
            storeBestClaim(holeClaims, claim);
        }
    }

    private void storeBestClaim(Map<Int2D, ClaimInfo> claims, ClaimInfo claim) {
        ClaimInfo existing = claims.get(claim.target);
        if (existing == null || estimateClaimDistance(claim) < estimateClaimDistance(existing)) {
            claims.put(claim.target, claim);
        }
    }

    private int estimateClaimDistance(ClaimInfo claim) {
        if (claim.senderLocation == null || claim.target == null) {
            return Integer.MAX_VALUE;
        }
        return Math.abs(claim.senderLocation.x - claim.target.x) + Math.abs(claim.senderLocation.y - claim.target.y);
    }

    private void rememberFuelStation(Int2D fuelStationLocation, double timestamp) {
        if (fuelStationLocation == null) {
            return;
        }
        permanentFuelStationLocation = new Int2D(fuelStationLocation.x, fuelStationLocation.y);
        sharedFuelStationLocation = new Int2D(fuelStationLocation.x, fuelStationLocation.y);
        sharedFuelStationTimestamp = timestamp;
    }

    private boolean isSelfMessage(Message message) {
        if (message instanceof CoordinationMessage) {
            Int2D senderLocation = ((CoordinationMessage) message).getSenderLocation();
            if (senderLocation != null && senderLocation.x == getX() && senderLocation.y == getY()) {
                return true;
            }
        }

        return name.equals(message.getFrom());
    }

    private void pruneStale(Map<Int2D, Double> map, double now) {
        List<Int2D> staleKeys = new ArrayList<>();
        for (Map.Entry<Int2D, Double> entry : map.entrySet()) {
            if (now - entry.getValue() > SHARED_INFO_TTL) {
                staleKeys.add(entry.getKey());
            }
        }
        for (Int2D key : staleKeys) {
            map.remove(key);
        }
    }

    private Int2D getKnownFuelStationLocation() {
        TWEntity visibleStation = getMemory().getClosestObjectInSensorRange(TWFuelStation.class);
        if (visibleStation != null) {
            Int2D station = new Int2D(visibleStation.getX(), visibleStation.getY());
            rememberFuelStation(station, getEnvironment().schedule.getTime());
            return station;
        }

        TWEntity rememberedStation = getMemoryModule().getClosestRememberedObject(TWFuelStation.class);
        if (rememberedStation != null) {
            Int2D station = new Int2D(rememberedStation.getX(), rememberedStation.getY());
            rememberFuelStation(station, getEnvironment().schedule.getTime());
            return station;
        }

        if (permanentFuelStationLocation != null) {
            return permanentFuelStationLocation;
        }

        return sharedFuelStationLocation;
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

    private int estimatePathCost(int startX, int startY, int goalX, int goalY) {
        if (startX == goalX && startY == goalY) {
            return 0;
        }

        int maxSearchDistance = getEnvironment().getxDimension() + getEnvironment().getyDimension();
        AstarPathGenerator astar = new AstarPathGenerator(getEnvironment(), this, maxSearchDistance);
        TWPath path = astar.findPath(startX, startY, goalX, goalY);
        if (path == null) {
            return Integer.MAX_VALUE;
        }
        return pathDistance(path);
    }

    private int pathDistance(TWPath path) {
        int steps = path.getpath().size();
        return Math.max(0, steps - 1);
    }

    private boolean shouldSeekFuel() {
        Int2D station = getKnownFuelStationLocation();
        if (station == null) {
            return fuelLevel < FUEL_LOW_THRESHOLD;
        }

        int toStationLowerBound = Math.abs(getX() - station.x) + Math.abs(getY() - station.y);
        return fuelLevel < FUEL_LOW_THRESHOLD || fuelLevel <= toStationLowerBound + FUEL_SAFETY_MARGIN;
    }

    private boolean shouldRefuelNow() {
        return fuelLevel < Parameters.defaultFuelLevel;
    }

    private String getClaimableGoalName(GoalMode mode) {
        if (mode == GoalMode.TILE || mode == GoalMode.SHARED_TILE) {
            return "TILE";
        }
        if (mode == GoalMode.HOLE || mode == GoalMode.SHARED_HOLE) {
            return "HOLE";
        }
        return null;
    }

    private boolean isSameTaskFamily(GoalMode first, GoalMode second) {
        if (first == null || second == null) {
            return false;
        }
        boolean firstTile = first == GoalMode.TILE || first == GoalMode.SHARED_TILE;
        boolean secondTile = second == GoalMode.TILE || second == GoalMode.SHARED_TILE;
        if (firstTile && secondTile) {
            return true;
        }
        boolean firstHole = first == GoalMode.HOLE || first == GoalMode.SHARED_HOLE;
        boolean secondHole = second == GoalMode.HOLE || second == GoalMode.SHARED_HOLE;
        if (firstHole && secondHole) {
            return true;
        }
        return first == second;
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

