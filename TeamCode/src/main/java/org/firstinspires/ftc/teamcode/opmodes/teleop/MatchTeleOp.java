package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;
import org.firstinspires.ftc.teamcode.util.RunLog;

/**
 * Match teleop. Subclasses supply the alliance; {@code configure()} is final and does the rest.
 */
public abstract class MatchTeleOp extends ForwardOpMode {

    protected Robot robot;

    protected abstract Alliance alliance();

    @Override
    protected final void configure() {
        robot = Robot.forTeleOp(hardwareMap, alliance(), loopId());
        DriverBindings.configure(robot, driver, operator);
    }

    @Override
    protected void logSignals(RunLog log) {
        robot.logSignals(log);
    }

    @Override
    public void run() {
        super.run();
        if (telemetryDue(250)) {
            telemetry.addData("alliance", robot.alliance);
            telemetry.addData("loop", "%.0f Hz", loopHz());
            telemetry.addData("pose", robot.drive.pose());
            telemetry.addData("Current RPM", robot.pollenShooter.getCurrentRpm());
            telemetry.addData("Target RPM", robot.pollenShooter.getTargetRpm());
            telemetry.update();
        }
    }
}
