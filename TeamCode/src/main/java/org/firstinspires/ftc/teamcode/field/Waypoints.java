package org.firstinspires.ftc.teamcode.field;

import java.nio.file.Watchable;

public final class Waypoints {

    // Shared by every auto.
    public static final Waypoint bottomAutoStart = Waypoint.red("hiveStart", 56, 9, 270);
    public static final Waypoint bottomHiveScore = Waypoint.red("bottomHiveScore", 56, 30, 270);
    public static final Waypoint goToSideFlower = Waypoint.red("goToSideFlower", 19, 50, 180);
    public static final Waypoint pickUpSideFlower = Waypoint.red("pickUpSideFlower", 11.5, 50, 180);
    public static final Waypoint scoreTopHive = Waypoint.red("topHiveScore", 57.5146, 129, 90);

    public static final Waypoint goToHiveFlower = Waypoint.red("goToHiveFlower", 48, 115, 90);
    public static final Waypoint pickUpHiveFlower = Waypoint.red("pickUpHiveFlower", 50, 128, 90);
    public static final Waypoint goToTeammate = Waypoint.red("goToTeammate", 30, 114, 180);
    public static final Waypoint pickUpTeammate = Waypoint.red("pickUpTeammate", 20.4521, 114, 180);
    public static final Waypoint parkFr = Waypoint.red("point13", 11.4875, 89.5021, 90);

    public static final Waypoint topAutoStart = Waypoint.red("topStart" ,62, 132, 90);


    // Solo auto only.
    public static final Waypoint scoreSecond = Waypoint.red("scoreSecond", 59, 114, 90);
    public static final Waypoint scoreTeammate = Waypoint.red("scoreTeammate", 59, 114, 90);
    public static final Waypoint goToPark = Waypoint.red("goToPark", 33.4, 106.4198, 90);
    public static final Waypoint downToPark = Waypoint.red("downToPark", 32.7125, 90, 90);

    //Starting Top Partner Auto only.
    public static final Waypoint curveToTopFlowerControl = Waypoint.red("curveToTopFlowerControl1",54.601, 91.375, 0);
    public static final Waypoint leaveTopFlower = Waypoint.red("leaveTopFlower", 31.2437, 113.9406, 180);
    public static final Waypoint backUpSideFlower = Waypoint.red("backUpSideFlower", 34, 47, 90);
    public static final Waypoint goBackUp = Waypoint.red("goBackUp", 34, 115, 90);
    public static final Waypoint getReadyToGoBackDown = Waypoint.red("getReadyToGoBackDown", 34, 115, 270);
    public static final Waypoint goDown = Waypoint.red("goDown" ,34, 20, 270);
    public static final Waypoint goToGarden = Waypoint.red("goToGarden", 9, 15, 270);
    public static final Waypoint pickUpGarden = Waypoint.red("pickUpGarden", 9, 13, 270);
    public static final Waypoint parkTopAuto = Waypoint.red("parkTopAuto", 14.3656, 92.3698, 270);
    public static final Waypoint curveToParkTop = Waypoint.red("curveToParkTop", 20.1625, 17.8271, 0);
}
