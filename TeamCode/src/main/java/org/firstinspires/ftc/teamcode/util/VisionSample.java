package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One camera reading, immutable. Angles are degrees, distances inches, {@code stalenessMs} the age
 * of the frame at the moment it was read.
 *
 * <p>Built by a {@link VisionCamera} once per loop. Copy methods follow {@code DriveInput}: start
 * from the targeting values, then add what the pipeline actually produced.
 */
public final class VisionSample {

    /**
     * One fiducial in the frame. {@code txDegrees} is positive to the right of the crosshair.
     *
     * <p>{@code xIn}, {@code yIn} and {@code zIn} are the tag's position in Limelight robot space:
     * X forward, Y right, Z up, in inches from the centre of the robot at floor level. They include
     * the camera's mounting offset only once it is entered in the Limelight web interface.
     */
    public static final class Tag {
        public final int id;
        public final double txDegrees;
        public final double tyDegrees;
        public final double area;

        /** Forward of the robot's centre, inches. */
        public final double xIn;

        /** Right of the robot's centre, inches. */
        public final double yIn;

        /** Up from the floor, inches. */
        public final double zIn;

        public Tag(
                int id,
                double txDegrees,
                double tyDegrees,
                double area,
                double xIn,
                double yIn,
                double zIn) {
            this.id = id;
            this.txDegrees = txDegrees;
            this.tyDegrees = tyDegrees;
            this.area = area;
            this.xIn = xIn;
            this.yIn = yIn;
            this.zIn = zIn;
        }
    }

    private static final VisionSample NONE = new VisionSample(false, 0, -1, 0, 0, 0);

    private final boolean valid;
    private final long stalenessMs;
    private final int pipeline;
    private final double tx;
    private final double ty;
    private final double ta;

    private List<Tag> tags = Collections.emptyList();

    public VisionSample(
            boolean valid, long stalenessMs, int pipeline, double tx, double ty, double ta) {
        this.valid = valid;
        this.stalenessMs = stalenessMs;
        this.pipeline = pipeline;
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
    }

    /** Nothing seen: invalid, no tags. */
    public static VisionSample none() {
        return NONE;
    }

    public VisionSample withTags(List<Tag> tags) {
        VisionSample copy = copy();
        copy.tags = Collections.unmodifiableList(new ArrayList<>(tags));
        return copy;
    }

    private VisionSample copy() {
        VisionSample copy = new VisionSample(valid, stalenessMs, pipeline, tx, ty, ta);
        copy.tags = tags;
        return copy;
    }

    /**
     * The camera had a target when the frame was captured. False also covers a camera that has not
     * been reached yet, which is why {@code getLatestResult() != null} is not a usable check.
     */
    public boolean isValid() {
        return valid;
    }

    public long stalenessMs() {
        return stalenessMs;
    }

    public int pipeline() {
        return pipeline;
    }

    /** Crosshair offsets, degrees, and target area as a percentage of the frame. */
    public double tx() {
        return tx;
    }

    public double ty() {
        return ty;
    }

    public double ta() {
        return ta;
    }

    public List<Tag> tags() {
        return tags;
    }

    public boolean sawTag(int id) {
        return tag(id) != null;
    }

    /** Horizontal offset to one tag in degrees, or NaN when that tag is not in the frame. */
    public double txForTag(int id) {
        Tag found = tag(id);
        return found == null ? Double.NaN : found.txDegrees;
    }

    /** One tag by ID, or null when it is not in the frame. */
    public Tag tag(int id) {
        for (int i = 0; i < tags.size(); i++) {
            if (tags.get(i).id == id) {
                return tags.get(i);
            }
        }
        return null;
    }
}
