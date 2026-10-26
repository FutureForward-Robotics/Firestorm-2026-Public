package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Hive;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.util.MotionSource;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.function.LongSupplier;

public class ShotCalculator {

    public static final double G = 386.09; // gravity
    public static final double DRAG_COEFFICIENT = 0.28;
    // Settings (values in the settings table)
    public double launchAngleDegrees; // θ
    public double launchForwardInches; // f
    public double launchLeftInches; // l (ell)
    public double effectiveHeightInches; // h
    public double rpmPerBallSpeed; // K
    public double airDrag; // k
    public double maxAimShiftInches;
    public double maxBearingDegrees = 90;
    public double entranceShiftPerInch;
    public double nearShotDistanceInches;

    //Results
    private boolean canShoot;
    private double robotHeading;
    private double wheelRpm;
    private double shooterDistance;
    private double targetCellY;
    private double bearingDeg;
    private double aimShiftInches;
    private double entranceShiftInches;
    private MotionSource motion;
    private Alliance alliance;
    private LongSupplier loopId;
    private long solvedLoop = -1;

    public ShotCalculator(MotionSource motion, Alliance alliance, LongSupplier loopId){
        this.motion = motion;
        this.alliance = alliance;
        this.loopId = loopId;
    }

    /**
     * The robot's POLLEN shooter. Only the launch angle has been measured. The launch point
     * offsets are 0 until they are measured, and the height and rpm settings are the simulator's
     * values until the field calibration replaces them.
     */
    public static ShotCalculator forPollen(MotionSource motion, Alliance alliance, LongSupplier loopId) {
        ShotCalculator pollenShot = new ShotCalculator(motion, alliance, loopId);
        pollenShot.launchAngleDegrees = 70;
        pollenShot.launchForwardInches = -8;
        pollenShot.launchLeftInches = 4;
        pollenShot.effectiveHeightInches = 49.61;
        pollenShot.rpmPerBallSpeed = 9.7;
        pollenShot.airDrag = 0.001213;
        pollenShot.maxAimShiftInches = 2;
        pollenShot.maxBearingDegrees = 55;
        pollenShot.entranceShiftPerInch = 0.13;
        pollenShot.nearShotDistanceInches = 44;
        return pollenShot;
    }

    /**
     * The robot's NECTAR shooter. NECTAR is heavier, so it slows the wheel more and needs more rpm
     * for the same ball speed. The same placeholders apply as for {@link #forPollen}.
     */
    public static ShotCalculator forNectar(MotionSource motion, Alliance alliance, LongSupplier loopId) {
        ShotCalculator nectarShot = new ShotCalculator(motion, alliance, loopId);
        nectarShot.launchAngleDegrees = 70;
        nectarShot.launchForwardInches = -8;
        nectarShot.launchLeftInches = -9;
        nectarShot.effectiveHeightInches = 49.61;
        nectarShot.rpmPerBallSpeed = 15.810;
        nectarShot.airDrag = 0.001212;
        return nectarShot;
    }

    private void solve() {
        long loop = loopId.getAsLong();
        if (solvedLoop == loop) {
            return;
        }
        solvedLoop = loop;
        Pose robot = motion.pose();
        double launchAngleRad = Math.toRadians(launchAngleDegrees);
        // Step 1: target cell
        double allianceX = Hive.cellX(alliance);
        double audienceY = Hive.AUDIENCE_CELL_Y;
        double farY = Hive.FAR_CELL_Y;

        // pythag theorem to find closest one
        double toAudience = Math.hypot(allianceX - robot.x(), audienceY - robot.y());
        double toFar = Math.hypot(allianceX - robot.x(), farY - robot.y());
        boolean cellOpensTowardAudience = toAudience <= toFar;
        targetCellY = cellOpensTowardAudience ? audienceY : farY;

        double inFrontInches = cellOpensTowardAudience ? targetCellY - robot.y() : robot.y() - targetCellY;
        double sidewaysInches = Math.abs(robot.x() - allianceX);
        double bearingRad = Math.atan2(sidewaysInches, inFrontInches);
        bearingDeg = Math.toDegrees(bearingRad);

        double lipCrossingOffset = clamp(Hive.LIP_SETBACK_IN * Math.tan(bearingRad), 0, maxAimShiftInches);
        aimShiftInches = -lipCrossingOffset * Math.signum(robot.x() - allianceX);

        double distanceToCell = Math.hypot(robot.x() - allianceX, robot.y() - targetCellY);
        entranceShiftInches =
                clamp(entranceShiftPerInch * (distanceToCell - nearShotDistanceInches), 0, Hive.LIP_SETBACK_IN);
        double towardEntranceY = cellOpensTowardAudience ? -1 : 1;

        Pose closestHive = new Pose(
                allianceX + aimShiftInches, targetCellY + towardEntranceY * entranceShiftInches);
        double closestDistance = Math.hypot(closestHive.x() - robot.x(), closestHive.y() - robot.y());

        // Step 2: fire direction
        double alpha = Math.asin(launchLeftInches / closestDistance);
        double fireAngle = Math.atan2(closestHive.y()-robot.y(), closestHive.x()-robot.x())+alpha;

        // Step 3: flat distance
        shooterDistance = closestDistance*Math.cos(alpha)+launchForwardInches;

        // Step 4: robot heading
        robotHeading = Field.normalize(fireAngle + Math.PI);

        // Step 5: range check
        boolean inRange = shooterDistance*Math.tan(launchAngleRad)>effectiveHeightInches;
        canShoot = inRange && bearingDeg <= maxBearingDegrees;
        if (!inRange) {
            wheelRpm = 0;
            return;
        }
        // Step 6: ideal launch speed

        double v = Math.sqrt((G * shooterDistance*shooterDistance) /
                (2*Math.pow(Math.cos(launchAngleRad), 2) *(shooterDistance*Math.tan(launchAngleRad)-effectiveHeightInches)));
        double vBall = v * (1 + DRAG_COEFFICIENT*(airDrag*shooterDistance)/Math.cos(launchAngleRad));

        // Step 7: wheel rpm
        wheelRpm = rpmPerBallSpeed * vBall;
    }

    public boolean canShoot(){
        solve();
        return canShoot;
    }

    public double robotHeading(){
        solve();
        return robotHeading;
    }

    public double wheelRpm(){
        solve();
        return wheelRpm;
    }

    /** How far the robot still has to turn to be on {@link #robotHeading()}, in radians. */
    public double headingError(){
        solve();
        return Field.normalize(robotHeading - motion.pose().heading());
    }

    /**
     * Records this shot in the run log. {@code name} is the chart group, such as "pollenShot".
     * Headings are logged in degrees, from -180 to 180.
     */
    public void logSignals(RunLog log, String name) {
        log.addFlag(name + ".canShoot", this::canShoot);
        log.addSignal(name + ".headingDeg", () -> Math.toDegrees(robotHeading()));
        log.addSignal(name + ".headingErrorDeg", () -> Math.toDegrees(headingError()));
        log.addSignal(name + ".wheelRpm", this::wheelRpm);
        log.addSignal(name + ".distance", () -> { solve(); return shooterDistance; });
        log.addSignal(name + ".targetCellY", () -> { solve(); return targetCellY; });
        log.addSignal(name + ".bearingDeg", () -> { solve(); return bearingDeg; });
        log.addSignal(name + ".aimShiftIn", () -> { solve(); return aimShiftInches; });
        log.addSignal(name + ".entranceShiftIn", () -> { solve(); return entranceShiftInches; });
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


}
