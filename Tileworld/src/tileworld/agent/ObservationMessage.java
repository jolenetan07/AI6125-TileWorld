package tileworld.agent;

import sim.util.Int2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Structured broadcast payload for sharing sensed tiles and holes.
 */
public class ObservationMessage extends Message {
    private final List<Int2D> tileLocations;
    private final List<Int2D> holeLocations;
    private final double timestamp;

    public ObservationMessage(String from, String to, List<Int2D> tileLocations, List<Int2D> holeLocations, double timestamp) {
        super(from, to, "OBSERVATION");
        this.tileLocations = new ArrayList<Int2D>(tileLocations);
        this.holeLocations = new ArrayList<Int2D>(holeLocations);
        this.timestamp = timestamp;
    }

    public List<Int2D> getTileLocations() {
        return Collections.unmodifiableList(tileLocations);
    }

    public List<Int2D> getHoleLocations() {
        return Collections.unmodifiableList(holeLocations);
    }

    public double getTimestamp() {
        return timestamp;
    }
}

