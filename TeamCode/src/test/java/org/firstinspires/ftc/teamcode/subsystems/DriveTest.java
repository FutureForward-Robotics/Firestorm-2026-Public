package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.junit.jupiter.api.Test;

// Poses taken from run 20261014-145241-TeleOp_Red's last loop.
class DriveTest {

    private static final Pose ON_FIELD = new Pose(102.44, 16.01, Math.toRadians(324.13));
    private static final Pose MOVED = new Pose(102.50, 16.00, Math.toRadians(324.10));
    private static final Velocity DRIFTING = new Velocity(0.0015, -0.0013, 0);

    @Test
    void aNormalReadingIsTrusted() {
        assertTrue(Drive.isTrustworthyReading(MOVED, DRIFTING, ON_FIELD, false));
    }

    @Test
    void anyReadingDuringAnInterruptIsRejected() {
        assertFalse(
                Drive.isTrustworthyReading(MOVED, DRIFTING, ON_FIELD, true),
                "the SDK may have handed back placeholder bytes for this read");
    }

    @Test
    void allZerosAwayFromTheOriginIsRejected() {
        assertFalse(
                Drive.isTrustworthyReading(Pose.zero(), Velocity.zero(), ON_FIELD, false),
                "a failed read without the interrupt returns the same zeros");
    }

    @Test
    void allZerosAtTheOriginIsTrusted() {
        assertTrue(
                Drive.isTrustworthyReading(Pose.zero(), Velocity.zero(), Pose.zero(), false),
                "a robot seeded at the origin and standing still really reads zero");
    }

    @Test
    void passingThroughTheOriginWhileMovingIsTrusted() {
        Velocity moving = new Velocity(12, 0, 0);
        assertTrue(Drive.isTrustworthyReading(Pose.zero(), moving, ON_FIELD, false));
    }
}
