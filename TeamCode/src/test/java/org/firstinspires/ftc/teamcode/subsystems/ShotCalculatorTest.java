package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.util.MotionSource;
import org.junit.jupiter.api.Test;

/** The six cases from the shooter aiming worksheet, and the once-per-loop rule. */
class ShotCalculatorTest {

    private static final double RPM_TOLERANCE = 1;
    private static final double HEADING_TOLERANCE_DEGREES = 0.05;

    // The test sets where the robot is and which loop it is.
    private Pose pose;
    private long loop;

    private final MotionSource motion =
            new MotionSource() {
                @Override
                public Pose pose() {
                    return pose;
                }

                @Override
                public double velocityX() {
                    return 0;
                }

                @Override
                public double velocityY() {
                    return 0;
                }
            };

    // The worksheet's example settings. The expected numbers below only hold for these.
    private ShotCalculator worksheetExample(Alliance alliance) {
        ShotCalculator shot = new ShotCalculator(motion, alliance, () -> loop);
        shot.launchAngleDegrees = 65;
        shot.launchForwardInches = 1.85;
        shot.launchLeftInches = 0;
        shot.effectiveHeightInches = 49.61;
        shot.rpmPerBallSpeed = 14.614;
        shot.airDrag = 0.001213;
        return shot;
    }

    @Test
    void caseA_baseCase() {
        pose = new Pose(59.25, 8.53, 0);
        ShotCalculator shot = worksheetExample(Alliance.RED);

        assertTrue(shot.canShoot());
        assertEquals(-90.00, Math.toDegrees(shot.robotHeading()), HEADING_TOLERANCE_DEGREES);
        assertEquals(3288.8, shot.wheelRpm(), RPM_TOLERANCE);
    }

    @Test
    void caseB_offsetLaunchPoint() {
        pose = new Pose(59.25, 8.53, 0);
        ShotCalculator shot = worksheetExample(Alliance.RED);
        shot.launchLeftInches = 4;

        assertEquals(-85.19, Math.toDegrees(shot.robotHeading()), HEADING_TOLERANCE_DEGREES);
        assertEquals(3287.7, shot.wheelRpm(), RPM_TOLERANCE);
    }

    @Test
    void caseC_nearerCellSelected() {
        pose = new Pose(59.25, 135.47, 0);
        ShotCalculator shot = worksheetExample(Alliance.RED);

        assertEquals(90.00, Math.toDegrees(shot.robotHeading()), HEADING_TOLERANCE_DEGREES);
        assertEquals(3288.8, shot.wheelRpm(), RPM_TOLERANCE);
    }

    @Test
    void caseD_angledShot() {
        pose = new Pose(40, 20, 0);
        ShotCalculator shot = worksheetExample(Alliance.RED);

        assertEquals(-117.98, Math.toDegrees(shot.robotHeading()), HEADING_TOLERANCE_DEGREES);
        assertEquals(3274.9, shot.wheelRpm(), RPM_TOLERANCE);
    }

    @Test
    void caseE_caseDForBlue() {
        pose = new Pose(104, 124, 0);
        ShotCalculator shot = worksheetExample(Alliance.BLUE);

        assertEquals(62.02, Math.toDegrees(shot.robotHeading()), HEADING_TOLERANCE_DEGREES);
        assertEquals(3274.9, shot.wheelRpm(), RPM_TOLERANCE);
    }

    @Test
    void caseF_tooCloseToShoot() {
        pose = new Pose(59.25, 40, 0);
        ShotCalculator shot = worksheetExample(Alliance.RED);

        assertFalse(shot.canShoot());
        assertEquals(0, shot.wheelRpm(), RPM_TOLERANCE);
    }

    @Test
    void solvesOncePerLoop() {
        pose = new Pose(59.25, 8.53, 0);
        ShotCalculator shot = worksheetExample(Alliance.RED);
        assertEquals(3288.8, shot.wheelRpm(), RPM_TOLERANCE);

        // Same loop: the answer doesn't change even though the robot has moved.
        pose = new Pose(40, 20, 0);
        assertEquals(3288.8, shot.wheelRpm(), RPM_TOLERANCE);

        // Next loop: solved again from the new pose.
        loop++;
        assertEquals(3274.9, shot.wheelRpm(), RPM_TOLERANCE);
    }
}
