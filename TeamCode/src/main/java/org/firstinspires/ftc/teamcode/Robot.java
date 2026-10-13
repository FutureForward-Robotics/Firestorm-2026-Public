package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.ScoringElement;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
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
    public final Shooter pollenShooter;
    public final Shooter nectarShooter;
    public final Intake intake;
    public final Indexer indexer;

    /** Auto. Starts from a known waypoint and clears the stored pose. */
    public Robot(HardwareMap hardwareMap, Alliance alliance, Waypoint start, Telemetry telemetry) {
        this.alliance = alliance;
        PoseStore.clear();
        this.drive = new Drive(hardwareMap, start.pose(alliance));
        pollenShooter = buildPollenShooter(hardwareMap);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        nectarShooter = new Shooter("nectarShooter", hardwareMap.get(DcMotorEx.class, "nectarShooter"), 0.0005);
        indexer = buildIndexer(hardwareMap);
    }

    /** Teleop. Resumes the pose auto left behind. */
    public Robot(HardwareMap hardwareMap, Alliance alliance, Telemetry telemetry) {
        this.alliance = alliance;
        this.drive = new Drive(hardwareMap);
        pollenShooter = buildPollenShooter(hardwareMap);
        nectarShooter = new Shooter("nectarShooter", hardwareMap.get(DcMotorEx.class, "nectarShooter"), 0.0005);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        indexer = buildIndexer(hardwareMap);
    }

    private static Indexer buildIndexer(HardwareMap hardwareMap) {
        return new Indexer(
                hardwareMap.get(DcMotorEx.class, "indexerBelt"),
                hardwareMap.get(Servo.class, "gatePollen"),
                hardwareMap.get(Servo.class, "gateNectar"));
    }

    public static Shooter buildPollenShooter(HardwareMap hardwareMap) {
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        return new Shooter("pollenShooter", shooterMotor, 0.0005);
    }

    public Shooter shooterFor(ScoringElement element) {
        return element == ScoringElement.POLLEN ? pollenShooter : nectarShooter;
    }
}
