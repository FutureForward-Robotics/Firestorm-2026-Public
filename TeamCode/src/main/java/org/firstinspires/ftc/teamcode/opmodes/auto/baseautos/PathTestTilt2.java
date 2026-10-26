package org.firstinspires.ftc.teamcode.opmodes.auto.baseautos;

import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.field.Waypoints;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;
import org.firstinspires.ftc.teamcode.util.RunLog;

public abstract class PathTestTilt2 extends ForwardOpMode {
    protected Robot robot;
    protected Route route;
    private Command auto;

    private static final Waypoint SIDE_FLOWER_TO_TOP_HIVE_CONTROL =
            Waypoint.red("sideFlowerToTopHiveControl", 31.688541666666673, 110.578125, 0);

    private static final long PICKUP_WAIT_MS = 2000;
    private static final long SPIN_UP_TIMEOUT_MS = 2000;

    public abstract Alliance getAlliance();

    @Override
    protected void configure() {
        robot = Robot.forAutonomous(hardwareMap, getAlliance(), Waypoints.bottomAutoStart, loopId());
        route = robot.drive.route(getAlliance(), Waypoints.bottomAutoStart);

        Path scorePreload = route.lineTo(Waypoints.bottomHiveScore);
        Path goToSideFlower = route.lineTo(Waypoints.goToSideFlower);
        Path pickSideFlower = route.lineTo(Waypoints.pickUpSideFlower);
        Path curveToTopHiveScore = route.curveTo(Waypoints.scoreTopHive, SIDE_FLOWER_TO_TOP_HIVE_CONTROL);
        Path goToTopFlower = route.lineTo(Waypoints.goToHiveFlower);
        Path pickUpTopFlower = route.lineTo(Waypoints.pickUpHiveFlower);
        Path scoreTopFlower = route.lineTo(Waypoints.scoreTopHive);
        Path park = route.continuousTo(Waypoints.goToPark, Waypoints.downToPark, Waypoints.parkFr);

        auto = new SequentialCommandGroup(
                robot.drive.follow(scorePreload),
                robot.drive.follow(goToSideFlower) ,
                robot.drive.follow(pickSideFlower),
                robot.drive.follow(curveToTopHiveScore),
                robot.drive.follow(goToTopFlower),
                robot.drive.follow(pickUpTopFlower),
                robot.drive.follow(scoreTopFlower),
                robot.drive.follow(park));

    }

    @Override
    protected void logSignals(RunLog log) {
        robot.logSignals(log);
    }


    /**
     * Runs once when Play is pressed. Scheduling in configure() would start the first path and its
     * timeout during init.
     */
    @Override
    public void preRun() {
        schedule(auto);
    }
}
