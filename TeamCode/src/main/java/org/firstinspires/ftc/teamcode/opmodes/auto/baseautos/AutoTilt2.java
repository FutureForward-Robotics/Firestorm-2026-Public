package org.firstinspires.ftc.teamcode.opmodes.auto.baseautos;

import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Waypoints;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;

public abstract class AutoTilt2 extends ForwardOpMode {
    protected Robot robot;
    protected Route route;
    private Command auto;

    public abstract Alliance getAlliance();

    @Override
    protected void configure() {
        robot = new Robot(hardwareMap, getAlliance(), Waypoints.bottomAutoStart, telemetry);
        route = robot.drive.route(getAlliance(), Waypoints.bottomAutoStart);

        Path scorePreload = route.lineTo(Waypoints.bottomHiveScore);
        Path goToSideFlower = route.lineTo(Waypoints.goToSideFlower);
        Path pickSideFlower = route.lineTo(Waypoints.pickUpSideFlower);
        Path curveToTopHiveScore = route.curveTo(Waypoints.scoreTopHive, Waypoint.red("controlOne", 31.688541666666673, 110.578125, 0));
        Path goToTopFlower = route.lineTo(Waypoints.goToHiveFlower);
        Path pickUpTopFlower = route.lineTo(Waypoints.pickUpHiveFlower);
        Path scoreTopFlower = route.lineTo(Waypoints.scoreTopHive);
        Path headToPark = route.lineTo(Waypoints.goToPark);
        Path lowerToPark = route.lineTo(Waypoints.downToPark);
        Path finalPark = route.lineTo(Waypoints.parkFr);

        auto = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        robot.shooterSmall.readyShooter(),
                        robot.drive.follow(scorePreload)
                ),
                robot.shooterSmall.atRPMCmd(),
                // robot transfer / shoots

                new ParallelCommandGroup(
                        robot.drive.follow(goToSideFlower),
                        robot.intake.intakeCmd(),
                        robot.shooterSmall.disableShooter()
                ),

                robot.drive.follow(pickSideFlower),
                new WaitCommand(2000),


                new ParallelCommandGroup(
                        robot.shooterSmall.readyShooter(),
                        robot.drive.follow(curveToTopHiveScore),
                        robot.intake.stopIntakeCmd()
                ),

                robot.shooterSmall.atRPMCmd(),
                // robot transfer / shoots

                new ParallelCommandGroup(
                        robot.drive.follow(goToTopFlower),
                        robot.intake.intakeCmd(),
                        robot.shooterSmall.disableShooter()
                ),

                robot.drive.follow(pickUpTopFlower),
                new WaitCommand(2000),

                new ParallelCommandGroup(
                        robot.shooterSmall.readyShooter(),
                        robot.drive.follow(scoreTopFlower),
                        robot.intake.stopIntakeCmd()
                ),

                robot.shooterSmall.atRPMCmd(),
                // robot transfer / shoots

                new ParallelCommandGroup(
                robot.drive.follow(headToPark),
                robot.intake.stopIntakeCmd()
                ),

                robot.drive.follow(lowerToPark),
                robot.drive.follow(finalPark)
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
