package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.fakes.FakeMotor;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.firstinspires.ftc.teamcode.field.ScoringElement;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

@RobotTest
class IndexerTest {

    private static final double EPS = 1e-9;

    private FakeMotor beltMotor;
    private FakeServo pollenGate;
    private FakeServo nectarGate;
    private Indexer indexer;

    @BeforeEach
    void buildIndexer(LoopRunner runner) {
        beltMotor = runner.motor("indexerBelt");
        pollenGate = runner.servo("gatePollen");
        nectarGate = runner.servo("gateNectar");

        // Start away from the HOLD values, so a test can't pass just because the fakes start at 0.
        beltMotor.setPower(0.5);
        pollenGate.setPosition(0.5);
        nectarGate.setPosition(0.5);

        indexer = new Indexer(beltMotor, pollenGate, nectarGate);
    }

    @Test
    void theFirstLoopStopsTheBeltAndClosesBothGates(LoopRunner runner) {
        runner.loop();

        assertEquals(0, beltMotor.getPower(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, pollenGate.getPosition(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, nectarGate.getPosition(), EPS);
    }

    @Test
    void loadRunsTheBeltWithBothGatesClosed(LoopRunner runner) {
        indexer.load().schedule();
        runner.loops(3);

        assertEquals(Indexer.LOAD_POWER, beltMotor.getPower(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, pollenGate.getPosition(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, nectarGate.getPosition(), EPS);
    }

    @Test
    void feedingPollenOpensOnlyThePollenGate(LoopRunner runner) {
        indexer.feed(ScoringElement.POLLEN).schedule();
        runner.loops(3);

        assertEquals(Indexer.FEED_POWER, beltMotor.getPower(), EPS);
        assertEquals(Indexer.GATE_OPEN_POSITION, pollenGate.getPosition(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, nectarGate.getPosition(), EPS);
    }

    @Test
    void feedingNectarOpensOnlyTheNectarGate(LoopRunner runner) {
        indexer.feed(ScoringElement.NECTAR).schedule();
        runner.loops(3);

        assertEquals(Indexer.FEED_POWER, beltMotor.getPower(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, pollenGate.getPosition(), EPS);
        assertEquals(Indexer.GATE_OPEN_POSITION, nectarGate.getPosition(), EPS);
    }

    @Test
    void switchingFromPollenToNectarClosesThePollenGate(LoopRunner runner) {
        indexer.feed(ScoringElement.POLLEN).schedule();
        runner.loops(3);
        indexer.feed(ScoringElement.NECTAR).schedule();
        runner.loops(3);

        assertEquals(Indexer.GATE_CLOSED_POSITION, pollenGate.getPosition(), EPS);
        assertEquals(Indexer.GATE_OPEN_POSITION, nectarGate.getPosition(), EPS);
    }

    @Test
    void holdAfterFeedingStopsTheBeltAndClosesTheGate(LoopRunner runner) {
        indexer.feed(ScoringElement.POLLEN).schedule();
        runner.loops(3);
        indexer.hold().schedule();
        runner.loops(3);

        assertEquals(0, beltMotor.getPower(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, pollenGate.getPosition(), EPS);
    }

    @Test
    void unjamRunsTheBeltBackwardsWithBothGatesClosed(LoopRunner runner) {
        indexer.feed(ScoringElement.NECTAR).schedule();
        runner.loops(3);
        indexer.unjam().schedule();
        runner.loops(3);

        assertEquals(-Indexer.FEED_POWER, beltMotor.getPower(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, pollenGate.getPosition(), EPS);
        assertEquals(Indexer.GATE_CLOSED_POSITION, nectarGate.getPosition(), EPS);
    }
}
