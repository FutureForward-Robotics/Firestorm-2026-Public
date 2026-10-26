package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.fakes.FakeMotor;
import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.junit.jupiter.api.Test;

@RobotTest
class ShooterTest {

    private static final double RPM_EPS = 1;
    // The fake motor has no inertia, so any kP makes it oscillate; feedforward alone is exact.
    private static final double KP = 0;

    // Full power spins the fake at MAX_RPM.
    private static final double TICKS_PER_SECOND_AT_FULL_POWER =
            Shooter.MAX_RPM * Shooter.TICKS_PER_REV / 60;

    private Shooter shooterOn(LoopRunner runner) {
        FakeMotor motor = runner.motor("pollenShooter", TICKS_PER_SECOND_AT_FULL_POWER);
        return new Shooter("pollenShooter", motor, KP);
    }

    @Test
    void idleDropsToIdleRpmAfterAShot(LoopRunner runner) {
        Shooter shooter = shooterOn(runner);

        shooter.spinUp(() -> 2500).schedule();
        runner.loops(5);
        shooter.idle().schedule();
        runner.loops(5);

        assertEquals(Shooter.IDLE_RPM, shooter.getCurrentRpm(), RPM_EPS);
    }

    @Test
    void spinUpFollowsTheSupplierAfterItIsScheduled(LoopRunner runner) {
        Shooter shooter = shooterOn(runner);
        double[] shotRpm = {2500};

        shooter.spinUp(() -> shotRpm[0]).schedule();
        runner.loops(5);
        shotRpm[0] = 3200;
        runner.loops(5);

        assertEquals(3200, shooter.getTargetRpm(), RPM_EPS);
        assertEquals(3200, shooter.getCurrentRpm(), RPM_EPS);
    }

    @Test
    void theOffsetIsAppliedOnce(LoopRunner runner) {
        Shooter shooter = shooterOn(runner);

        shooter.offsetDown().schedule();
        shooter.offsetDown().schedule();
        runner.loops(1);
        shooter.spinUp(() -> 2500).schedule();
        runner.loops(5);

        assertEquals(2450, shooter.getTargetRpm(), RPM_EPS);
        assertEquals(2450, shooter.getCurrentRpm(), RPM_EPS);
    }

    @Test
    void atSpeedCountsTheOffset(LoopRunner runner) {
        Shooter shooter = shooterOn(runner);

        for (int press = 0; press < 4; press++) {
            shooter.offsetDown().schedule();
        }
        runner.loops(1);
        shooter.spinUp(() -> 2500).schedule();
        runner.loops(5);

        assertTrue(shooter.atSpeed(), "settled at 2400 with a -100 offset");
    }

    @Test
    void theOffsetDoesNotChangeIdle(LoopRunner runner) {
        Shooter shooter = shooterOn(runner);

        shooter.offsetDown().schedule();
        shooter.idle().schedule();
        runner.loops(5);

        assertEquals(Shooter.IDLE_RPM, shooter.getCurrentRpm(), RPM_EPS);
    }
}
