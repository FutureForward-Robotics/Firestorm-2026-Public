package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.ForwardOpMode;
import org.firstinspires.ftc.teamcode.opmodes.teleop.DriverBindings;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.LimelightCamera;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.firstinspires.ftc.teamcode.util.VisionSample;

import java.util.List;

/**
 * Drives normally and shows what the camera sees: whether there is a fresh target, which tags are
 * in the frame, and how old the frame is. Use it to confirm the pipeline and the tag IDs.
 *
 * <p>{@link Vision} is a subsystem, so its signals are already in the run log; there is nothing to
 * register here.
 */
@TeleOp(name = "Vision Check", group = "test")
public class VisionCheck extends ForwardOpMode {

    private Robot robot;
    private Vision vision;

    @Override
    protected void configure() {
        robot = Robot.forTeleOp(hardwareMap, Alliance.RED, loopId());
        vision = new Vision(new LimelightCamera(hardwareMap, "limelight"),Alliance.RED);

        DriverBindings.configure(robot, driver, operator);
    }

    @Override
    protected void logSignals(RunLog log) {
        robot.logSignals(log);
    }

    @Override
    public void run() {
        super.run();
        if (!telemetryDue(250)) {
            return;
        }
        telemetry.addData("loop", "%.0f Hz", loopHz());
        telemetry.addData("hasTarget", vision.hasTarget());
        telemetry.addData("tags", tagIds(vision.sample().tags()));
        telemetry.addData("staleness", "%d ms", vision.sample().stalenessMs());
        telemetry.update();
    }

    private static String tagIds(List<VisionSample.Tag> tags) {
        if (tags.isEmpty()) {
            return "none";
        }
        StringBuilder ids = new StringBuilder();
        for (int i = 0; i < tags.size(); i++) {
            if (i > 0) {
                ids.append(", ");
            }
            ids.append(tags.get(i).id);
        }
        return ids.toString();
    }
}
