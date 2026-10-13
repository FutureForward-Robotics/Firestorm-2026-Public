package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Hive;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.util.MotionSource;

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

    //Results
    private boolean canShoot;
    private double robotHeading;
    private double wheelRpm;
    private MotionSource motion;
    private Alliance alliance;
    private LongSupplier loopId;
    private long solvedLoop = -1;

    public ShotCalculator(MotionSource motion, Alliance alliance, LongSupplier loopId){
        this.motion = motion;
        this.alliance = alliance;
        this.loopId = loopId;
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
        Pose closestHive = toAudience <= toFar ? new Pose(allianceX, audienceY) : new Pose(allianceX, farY);
        double closestDistance = Math.min(toAudience, toFar);

        // Step 2: fire direction
        double alpha = Math.asin(launchLeftInches / closestDistance);
        double fireAngle = Math.atan2(closestHive.y()-robot.y(), closestHive.x()-robot.x())+alpha;

        // Step 3: flat distance
        double shooterDistance = closestDistance*Math.cos(alpha)+launchForwardInches;

        // Step 4: robot heading
        robotHeading = Field.normalize(fireAngle + Math.PI);

        // Step 5: range check
        canShoot = shooterDistance*Math.tan(launchAngleRad)>effectiveHeightInches;
        if (!canShoot) {
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


}
