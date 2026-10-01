package org.firstinspires.ftc.teamcode.field;

/** HIVE CELL AprilTags and heights. */
public final class Hive {

    private static int[] RED_FAR = {30, 31, 32, 33};
    private static int[] RED_AUDIENCE = {34, 35, 36, 37};

    private static int[] BLUE_FAR = {42, 43, 44, 45};
    private static int[] BLUE_AUDIENCE = {38, 39, 40, 41};

    private Hive() {}

    /** A CELL of one alliance's HIVE, named by which field wall it is closest to. */
    public enum Cell { FAR, AUDIENCE }

    /** Tag z above this is on the upward CELL, inches. Placeholder until measured. */
    public static final double UP_CUTOFF_IN = 42.0;

    /** The four tag IDs on one CELL, in cluster order. */
    public static int[] tagIds(Alliance alliance, Cell cell) {
        if (alliance.isRed()) {
            if (cell == Cell.FAR)
                return RED_FAR;

            if (cell == Cell.AUDIENCE)
                return RED_AUDIENCE;
        } else {
            if (cell == Cell.FAR)
                return BLUE_FAR;

            if (cell == Cell.AUDIENCE)
                return BLUE_AUDIENCE;
        }

        return null;
    }

    /** Which of this alliance's CELLS the tag is on, or null when it isn't one of ours. */
    public static Cell cellOf(Alliance alliance, int tagId) {
        if (alliance.isRed()) {
            if (tagId >= 30 && tagId <= 33) {
                return Cell.FAR;
            } else if (tagId >= 34 && tagId <= 37) {
                return Cell.AUDIENCE;
            }
        } else {
            if (tagId >= 42 && tagId <= 45) {
                return Cell.FAR;
            } else if (tagId >= 38 && tagId <= 41) {
                return Cell.AUDIENCE;
            }
        }
        
        return null;
    }
}