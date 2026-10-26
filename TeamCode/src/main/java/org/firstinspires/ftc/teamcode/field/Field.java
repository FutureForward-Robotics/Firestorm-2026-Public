package org.firstinspires.ftc.teamcode.field;

/**
 * Field geometry.
 *
 * <p>{@link #SYMMETRY} is a per-season setting: check whether this year's field is a reflection or
 * a 180 degree rotation before writing any waypoints. It only supplies the default blue value;
 * per-side differences are pinned on the individual {@link Waypoint}.
 */
public final class Field {

    private Field() {}

    public static final double WIDTH_IN = 144.0;

    /**
     * Set for the season. 2025-26 DECODE was {@link FieldSymmetry#MIRROR_X}. 2026-27 BIOBUZZ is
     * {@link FieldSymmetry#ROTATIONAL}: the loading zones, gardens and flowers sit 180 degrees apart
     * (manual Figures 9-2 and 10-2).
     */
    public static final FieldSymmetry SYMMETRY = FieldSymmetry.ROTATIONAL;

    /**
     * Wraps an angle into [-pi, pi). This is the convention for a heading <em>difference</em>: the
     * sign is the direction to turn and the magnitude is the shortest path.
     *
     * <p>Pedro stores an absolute heading in a {@code Pose} as [0, 2pi) instead, so {@code
     * pose.heading()} is never negative. Normalizing a value before handing it to a {@code Pose}
     * does nothing, because the constructor re-wraps it. Pedro's own {@code Angle} class keeps both
     * ranges for the same reason, and its follower computes every error with the signed one.
     */
    public static double normalize(double rad) {
        double a = (rad + Math.PI) % (2 * Math.PI);
        if (a < 0) {
            a += 2 * Math.PI;
        }
        return a - Math.PI;
    }
}
