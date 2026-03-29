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

public class GreedyReactiveAgent extends TWAgent {

    private transient TWPath currentPath = null;

    public GreedyReactiveAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
    }

    private String name;

    @Override
    protected TWThought think() {
        // Check if at a tile and can pick up
        TWEntity obj = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
        if (obj instanceof TWTile && carriedTiles.size() < 3) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }
        // Check if at a hole and has tile
        else if (obj instanceof TWHole && hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        // Else, move towards nearest target
        else {
            TWEntity target = hasTile() ? getMemory().getClosestObjectInSensorRange(TWHole.class) : getMemory().getClosestObjectInSensorRange(TWTile.class);
            if (target != null) {
                // Generate path if none or finished
                if (currentPath == null || !currentPath.hasNext()) {
                    AstarPathGenerator astar = new AstarPathGenerator(getEnvironment(), this, 50);
                    currentPath = astar.findPath(getX(), getY(), target.getX(), target.getY());
                }
                if (currentPath != null && currentPath.hasNext()) {
                    TWPathStep step = currentPath.popNext();
                    TWDirection dir = step.getDirection();
                    return new TWThought(TWAction.MOVE, dir);
                }
            }
            // No target or path, move randomly
            TWDirection randomDir = TWDirection.values()[getEnvironment().random.nextInt(5)];
            return new TWThought(TWAction.MOVE, randomDir);
        }
    }

    @Override
    protected void act(TWThought thought) {
        switch (thought.getAction()) {
            case MOVE:
                try {
                    move(thought.getDirection());
                } catch (CellBlockedException ex) {
                    // Path blocked, void path and replan next think
                    currentPath = null;
                }
                break;
            case PICKUP:
                TWEntity obj = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
                if (obj instanceof TWTile) {
                    pickUpTile((TWTile) obj);
                }
                break;
            case PUTDOWN:
                TWEntity obj2 = (TWEntity) getEnvironment().getObjectGrid().get(getX(), getY());
                if (obj2 instanceof TWHole) {
                    putTileInHole((TWHole) obj2);
                }
                break;
            case REFUEL:
                refuel();
                break;
        }
    }

    @Override
    public String getName() {
        return name;
    }
}
