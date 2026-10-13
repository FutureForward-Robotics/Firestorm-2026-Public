package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Hive;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.firstinspires.ftc.teamcode.util.VisionCamera;
import org.firstinspires.ftc.teamcode.util.VisionSample;

/**
 * Camera read once per loop.
 *
 * <p>The camera is a network round-trip, so {@link #sense()} takes exactly one reading and every
 * getter answers from it. Reading the device per question instead is how two values asked for in
 * the same loop end up describing different frames.
 *
 * <p>It also finds which of our HIVE CELLS is facing up, from the height of its tags, and where it
 * is relative to the robot. That is worked out once in {@link #sense()} so every getter describes
 * the same frame.
 *
 * <p>{@link #act()} is empty: a camera has no outputs.
 */
public final class Vision extends ForwardSubsystem {

    /** A frame older than this is ignored. One control loop at 50 Hz is 20 ms. */
    public static final long MAX_STALENESS_MS = 100;

    private final VisionCamera camera;
    private Alliance alliance;

    private VisionSample sample = VisionSample.none();

    private Hive.Cell upCell;
    private double bearingRad = Double.NaN;
    private double heightIn = Double.NaN;
    private double distanceIn = Double.NaN;

    public Vision(VisionCamera camera, Alliance alliance) {
        this.camera = camera;
        this.alliance = alliance;
    }

    public Vision(VisionCamera camera) {
        this(camera,Alliance.RED);
    }

    /**
     * Averages the visible tags of each of our CELLS, looked up by their known IDs, and keeps the
     * highest CELL above {@link Hive#UP_CUTOFF_IN}. Tags that are not ours are skipped. Every
     * result is null or NaN when there is no answer: a bearing of 0 means straight ahead.
     */
    @Override
    public void sense() {
        sample = camera.read();

        // Clear last loop's answer first, so a frame with no upward CELL does not keep the old one.
        upCell = null;
        bearingRad = Double.NaN;
        heightIn = Double.NaN;
        distanceIn = Double.NaN;

        if (!hasTarget()) {
            return;
        }

        // One running total per CELL, indexed by Cell.ordinal().
        Hive.Cell[] cells = Hive.Cell.values();
        double[] x = new double[cells.length];
        double[] y = new double[cells.length];
        double[] z = new double[cells.length];
        int[] count = new int[cells.length];

        for (VisionSample.Tag tag : sample.tags()) {
            Hive.Cell cell = Hive.cellOf(alliance, tag.id);
            if (cell == null) {
                continue;
            }
            int i = cell.ordinal();
            x[i] += tag.xIn;
            y[i] += tag.yIn;
            z[i] += tag.zIn;
            count[i]++;
        }

        int up = -1;
        double upZ = Hive.UP_CUTOFF_IN;
        for (int i = 0; i < cells.length; i++) {
            if (count[i] > 0 && z[i] / count[i] > upZ) {
                up = i;
                upZ = z[i] / count[i];
            }
        }

        if (up >= 0) {
            setUpCell(cells[up], x[up] / count[up], y[up] / count[up], upZ);
        }
    }

    /** Records the upward CELL from its averaged tag position, inches in robot space. */
    private void setUpCell(Hive.Cell cell, double x, double y, double z) {
        upCell = cell;
        bearingRad = Math.atan2(-y, x);
        heightIn = z;
        distanceIn = Math.hypot(x, y);
    }

    @Override
    public void act() {}

    /** This loop's reading, whether or not it is fresh. */
    public VisionSample sample() {
        return sample;
    }

    /** A target was seen in a frame new enough to act on. */
    public boolean hasTarget() {
        return sample.isValid() && sample.stalenessMs() <= MAX_STALENESS_MS;
    }

    /** Crosshair offset in degrees, positive to the right, or NaN with no fresh target. */
    public double tx() {
        return hasTarget() ? sample.tx() : Double.NaN;
    }

    public double ty() {
        return hasTarget() ? sample.ty() : Double.NaN;
    }

    public boolean sawTag(int id) {
        return hasTarget() && sample.sawTag(id);
    }

    /** Offset to one tag in degrees, or NaN when it is not in a fresh frame. */
    public double txForTag(int id) {
        return hasTarget() ? sample.txForTag(id) : Double.NaN;
    }

    /** Which of our CELLS is facing up, or null when unknown. */
    public Hive.Cell upCell() {
        return upCell;
    }
    public boolean isCellUp() {
        if(heightIn>Hive.UP_CUTOFF_IN){
            return true;
        }
        return false;
    }

    /**
     * Direction from the robot's centre to the upward CELL, radians, counter-clockwise positive, or
     * NaN when unknown.
     */
    public double bearingRad() {
        return bearingRad;
    }

    /** Height of the upward CELL's tags above the floor, inches, or NaN when unknown. */
    public double heightIn() {
        return heightIn;
    }

    /** Horizontal distance from the robot's centre to the upward CELL, inches, or NaN. */
    public double distanceIn() {
        return distanceIn;
    }

    public void pipeline(int index) {
        camera.pipeline(index);
    }

    @Override
    public void logSignals(RunLog log) {
        log.addFlag("vision.hasTarget", this::hasTarget);
        log.addSignal("vision.stalenessMs", () -> sample.stalenessMs());
        log.addSignal("vision.tx", this::tx);
        log.addSignal("vision.ty", this::ty);
        log.addSignal("vision.tags", () -> sample.tags().size());
        log.addSignal("vision.upCell", () -> upCell == null ? -1 : upCell.ordinal());
        log.addSignal("vision.bearingRad", this::bearingRad);
        log.addSignal("vision.heightIn", this::heightIn);
        log.addSignal("vision.distanceIn", this::distanceIn);
    }
}
