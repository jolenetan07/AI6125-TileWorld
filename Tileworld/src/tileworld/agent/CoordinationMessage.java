package tileworld.agent;

import sim.util.Int2D;

import java.util.List;

/**
 * Broadcast message used by coordinated agents.
 *
 * Extends {@link ObservationMessage} so existing communication-oriented agents
 * can still consume the shared tile/hole observations, while coordinated agents
 * additionally read teammate position, target intention, and fuel station info.
 */
public class CoordinationMessage extends ObservationMessage {
    private final Int2D senderLocation;
    private final Int2D intendedTarget;
    private final String intendedGoal;
    private final Int2D fuelStationLocation;

    public CoordinationMessage(String from,
                               String to,
                               List<Int2D> tileLocations,
                               List<Int2D> holeLocations,
                               double timestamp,
                               Int2D senderLocation,
                               Int2D intendedTarget,
                               String intendedGoal,
                               Int2D fuelStationLocation) {
        super(from, to, tileLocations, holeLocations, timestamp);
        this.senderLocation = senderLocation == null ? null : new Int2D(senderLocation.x, senderLocation.y);
        this.intendedTarget = intendedTarget == null ? null : new Int2D(intendedTarget.x, intendedTarget.y);
        this.intendedGoal = intendedGoal;
        this.fuelStationLocation = fuelStationLocation == null ? null : new Int2D(fuelStationLocation.x, fuelStationLocation.y);
    }

    public Int2D getSenderLocation() {
        return senderLocation;
    }

    public Int2D getIntendedTarget() {
        return intendedTarget;
    }

    public String getIntendedGoal() {
        return intendedGoal;
    }

    public Int2D getFuelStationLocation() {
        return fuelStationLocation;
    }
}

