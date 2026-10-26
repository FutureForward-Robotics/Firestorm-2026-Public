package org.firstinspires.ftc.teamcode.opmodes.auto.baseautos;

import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Waypoints;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;
import org.firstinspires.ftc.teamcode.util.RunLog;

public abstract class AutoPartnerTop extends ForwardOpMode {
    protected Robot robot;
    protected Route route;
    private Command auto;

    private static final long PICKUP_WAIT_MS = 2000;
    private static final long SPIN_UP_TIMEOUT_MS = 2000;

    public abstract Alliance getAlliance();

    @Override
    protected void configure() {
        robot = Robot.forAutonomous(hardwareMap, getAlliance(), Waypoints.topAutoStart, loopId());
        route = robot.drive.route(getAlliance(), Waypoints.topAutoStart);

        Path pickUpTopFlower = route.curveTo(Waypoints.pickUpHiveFlower, Waypoints.curveToTopFlowerControl);
        Path leaveTopFlower = route.lineTo(Waypoints.leaveTopFlower);
        Path goToSideFlower = route.lineTo(Waypoints.goToSideFlower);
        Path pickUpSideFlower = route.lineTo(Waypoints.pickUpSideFlower);
        Path backUpSideFlower = route.lineTo(Waypoints.backUpSideFlower);
        Path goBackUp = route.lineTo(Waypoints.goBackUp);
        Path scoreTopFlower = route.lineTo(Waypoints.scoreTopHive);
        Path getReadyGoBackDown = route.lineTo(Waypoints.getReadyToGoBackDown);
        Path goBackDown = route.lineTo(Waypoints.goDown);
        Path goToGarden = route.lineTo(Waypoints.goToGarden);
        Path pickUpGarden = route.lineTo(Waypoints.pickUpGarden);
        Path parkTopAuto = route.curveTo(Waypoints.parkTopAuto, Waypoints.curveToParkTop);
        Path scoreGarden = route.lineTo(Waypoints.bottomHiveScore);
        Path park = route.continuousTo(Waypoints.goToPark, Waypoints.downToPark, Waypoints.parkFr);

        auto = new SequentialCommandGroup(
                new SequentialCommandGroup(
                robot.pollenShooter.spinUp(),
                robot.intake.stop(),
                robot.pollenShooter.waitForSpeed()
                        //indexer
                ),
                robot.intake.start(),
                robot.drive.follow(pickUpTopFlower),
                robot.intake.stop(),
                robot.drive.follow(leaveTopFlower),
                collect(goToSideFlower, pickUpSideFlower),
                robot.drive.follow(backUpSideFlower),
                robot.drive.follow(goBackUp),
                scoreFrom(scoreTopFlower),
                robot.pollenShooter.off(),
                robot.drive.follow(park)
        );

    }

    @Override
    protected void logSignals(RunLog log) {
        robot.logSignals(log);
    }

    private Command scoreFrom(Path path){
        return new SequentialCommandGroup(
                robot.pollenShooter.spinUp(),
                robot.intake.stop(),
                robot.drive.follow(path),
                robot.pollenShooter.waitForSpeed()
        );
    }

    private Command collect(Path approach, Path pickup){
        return new SequentialCommandGroup(
                robot.pollenShooter.idle(),
                robot.intake.start(),
                robot.drive.follow(approach),
                robot.drive.follow(pickup),
                new WaitCommand(PICKUP_WAIT_MS)
        );
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
