package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.PoseStore;

/**
 * Composition root. Builds every subsystem once and holds the alliance for the match. OpModes
 * construct one and add bindings.
 */
public final class Robot {

    public final Alliance alliance;
    public final Drive drive;
    public final Shooter shooterSmall;
    public final Intake intake;

    /** Auto. Starts from a known waypoint and clears the stored pose. */
    public Robot(HardwareMap hardwareMap, Alliance alliance, Waypoint start, Telemetry telemetry) {
        this.alliance = alliance;
        PoseStore.clear();
        this.drive = new Drive(hardwareMap, start.pose(alliance));
        shooterSmall = new Shooter(hardwareMap.get(DcMotorEx.class, "shooterSmall"), 0.02);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));

    }

    /** Teleop. Resumes the pose auto left behind. */
    public Robot(HardwareMap hardwareMap, Alliance alliance, Telemetry telemetry) {
        this.alliance = alliance;
        this.drive = new Drive(hardwareMap);
        shooterSmall = new Shooter(hardwareMap.get(DcMotorEx.class, "shooterSmall"), 0.02);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
    }
}
