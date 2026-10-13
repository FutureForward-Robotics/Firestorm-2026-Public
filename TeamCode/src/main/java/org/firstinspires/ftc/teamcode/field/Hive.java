package org.firstinspires.ftc.teamcode.field;

/** HIVE CELL AprilTags and heights. */
public final class Hive {

    private static final int[] RED_FAR = {30, 31, 32, 33};
    private static final int[] RED_AUDIENCE = {34, 35, 36, 37};

    private static final int[] BLUE_FAR = {42, 43, 44, 45};
    private static final int[] BLUE_AUDIENCE = {38, 39, 40, 41};


    public static final double RED_HIVE_X = 59.25;
    public static final double BLUE_HIVE_X = 84.75;
    public static final double AUDIENCE_CELL_Y = 56.23;
    public static final double FAR_CELL_Y = 87.77;

    private Hive() {}

    /** A CELL of one alliance's HIVE, named by which field wall it is closest to. */
    public enum Cell { FAR, AUDIENCE }

    /** Tag z above this is on the upward CELL, inches. Placeholder until measured. */
    public static final double UP_CUTOFF_IN = 42.0;

    /** The four tag IDs on one CELL, in cluster order. The caller gets its own copy. */
    public static int[] tagIds(Alliance alliance, Cell cell) {
        int[] ids = ids(alliance, cell);
        return ids == null ? null : ids.clone();
    }

    /** Which of this alliance's CELLS the tag is on, or null when it isn't one of ours. */
    public static Cell cellOf(Alliance alliance, int tagId) {
        if (contains(ids(alliance, Cell.FAR), tagId)) {
            return Cell.FAR;
        }
        if (contains(ids(alliance, Cell.AUDIENCE), tagId)) {
            return Cell.AUDIENCE;
        }
        return null;
    }

    private static int[] ids(Alliance alliance, Cell cell) {
        if (cell == Cell.FAR) {
            return alliance.isRed() ? RED_FAR : BLUE_FAR;
        }
        if (cell == Cell.AUDIENCE) {
            return alliance.isRed() ? RED_AUDIENCE : BLUE_AUDIENCE;
        }
        return null;
    }

    private static boolean contains(int[] ids, int tagId) {
        for (int i = 0; i < ids.length; i++) {
            if (ids[i] == tagId) {
                return true;
            }
        }
        return false;
    }

    public static double cellX(Alliance alliance) {
        return alliance.isRed() ? RED_HIVE_X : BLUE_HIVE_X;
    }
}
