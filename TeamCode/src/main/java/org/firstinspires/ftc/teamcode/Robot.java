package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.ScoringElement;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.field.Waypoints;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.PoseStore;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.function.LongSupplier;

/**
 * Composition root. Builds every subsystem once and holds the alliance for the match. OpModes
 * get one from {@link #forAutonomous} or {@link #forTeleOp} and add bindings.
 */
public final class Robot {

    public final Alliance alliance;
    public final Drive drive;
    public final Shooter pollenShooter;
    public final ShotCalculator pollenCalculator;
    public final Shooter nectarShooter;
    public final Intake intake;
    public final Indexer indexer;

    private static final double SHOOTER_KP = 0.003;
    private static final long BATTERY_LOG_INTERVAL_MS = 500;

    private final VoltageSensor batteryVoltageSensor;

    public static Robot forAutonomous(
            HardwareMap hardwareMap, Alliance alliance, Waypoint start, LongSupplier loopId) {
        PoseStore.clear();
        Drive drive = new Drive(hardwareMap, start.pose(alliance));
        return new Robot(hardwareMap, alliance, drive, loopId);
    }

    public static Robot forTeleOp(HardwareMap hardwareMap, Alliance alliance, LongSupplier loopId) {
        Pose poseWithoutAutonomous = Waypoints.bottomAutoStart.pose(alliance);
        Drive drive = new Drive(hardwareMap, PoseStore.loadOr(poseWithoutAutonomous));
        return new Robot(hardwareMap, alliance, drive, loopId);
    }

    private Robot(HardwareMap hardwareMap, Alliance alliance, Drive drive, LongSupplier loopId) {
        this.alliance = alliance;
        this.drive = drive;

        // The SDK lists the battery voltage sensors without names. Every hub has one.
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        pollenShooter = buildShooter(hardwareMap, "pollenShooter", DcMotorSimple.Direction.REVERSE);
        nectarShooter = buildShooter(hardwareMap, "nectarShooter", DcMotorSimple.Direction.FORWARD);
        pollenCalculator = ShotCalculator.forPollen(drive, alliance, loopId);

        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
    }

    private static Shooter buildShooter(
            HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, name);
        shooterMotor.setDirection(direction);
        return new Shooter(name, shooterMotor, SHOOTER_KP);
    }

    /**
     * Run log signals that no subsystem owns. Call from the OpMode's {@code logSignals}. Battery
     * voltage is a slow signal because reading it is a round trip to the hub.
     */
    public void logSignals(RunLog log) {
        pollenCalculator.logSignals(log, "pollenShot");
        log.addSlowSignal("battery.volts", batteryVoltageSensor::getVoltage, BATTERY_LOG_INTERVAL_MS);
    }

    public Shooter shooterFor(ScoringElement element) {
        return element == ScoringElement.POLLEN ? pollenShooter : nectarShooter;
    }
}
