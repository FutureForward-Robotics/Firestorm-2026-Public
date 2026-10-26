package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.fakes.FakeCRServo;
import org.firstinspires.ftc.teamcode.fakes.FakeMotor;
import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.firstinspires.ftc.teamcode.field.ScoringElement;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

// Gate assertions are left out until the pollen gate's CRServo control is settled.
@RobotTest
class IndexerTest {

    private static final double EPS = 1e-9;

    private FakeMotor beltMotor;
    private Indexer indexer;

    @BeforeEach
    void buildIndexer(LoopRunner runner) {
        beltMotor = runner.motor(Indexer.BELT_MOTOR_NAME);
        runner.register(Indexer.POLLEN_SERVO_NAME, new FakeCRServo(Indexer.POLLEN_SERVO_NAME));

        // Start away from the HOLD value, so a test can't pass just because the fake starts at 0.
        beltMotor.setPower(0.5);

        indexer = new Indexer(runner.hardwareMap());
    }

    @Test
    void theFirstLoopStopsTheBelt(LoopRunner runner) {
        runner.loop();

        assertEquals(0, beltMotor.getPower(), EPS);
    }

    @Test
    void loadRunsTheBeltAtLoadPower(LoopRunner runner) {
        indexer.load().schedule();
        runner.loops(3);

        assertEquals(Indexer.LOAD_POWER, beltMotor.getPower(), EPS);
    }

    @Test
    void feedingPollenRunsTheBeltAtFeedPower(LoopRunner runner) {
        indexer.feed(ScoringElement.POLLEN).schedule();
        runner.loops(3);

        assertEquals(Indexer.FEED_POWER, beltMotor.getPower(), EPS);
    }

    @Test
    void feedingNectarRunsTheBeltAtFeedPower(LoopRunner runner) {
        indexer.feed(ScoringElement.NECTAR).schedule();
        runner.loops(3);

        assertEquals(Indexer.FEED_POWER, beltMotor.getPower(), EPS);
    }

    @Test
    void holdAfterFeedingStopsTheBelt(LoopRunner runner) {
        indexer.feed(ScoringElement.POLLEN).schedule();
        runner.loops(3);
        indexer.hold().schedule();
        runner.loops(3);

        assertEquals(0, beltMotor.getPower(), EPS);
    }

    @Test
    void unjamRunsTheBeltBackwards(LoopRunner runner) {
        indexer.feed(ScoringElement.NECTAR).schedule();
        runner.loops(3);
        indexer.unjam().schedule();
        runner.loops(3);

        assertEquals(-Indexer.FEED_POWER, beltMotor.getPower(), EPS);
    }
}
