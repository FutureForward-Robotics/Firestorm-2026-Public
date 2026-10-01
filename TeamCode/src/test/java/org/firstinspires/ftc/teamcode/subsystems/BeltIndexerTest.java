package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.firstinspires.ftc.teamcode.subsystems.indexers.BeltIndexer;
import org.junit.jupiter.api.Test;

@RobotTest
public class BeltIndexerTest {

    @Test
    void startMotor(LoopRunner runner) {
        runner.motor("belt_indexer");
        BeltIndexer beltIndexer = new BeltIndexer(runner.hardwareMap(),null);

        // action
        beltIndexer.feed().schedule();
        // time to run
        runner.loops(10);

        //verification
        assertEquals(1, beltIndexer.getIndexerPower(), 0.01);
    }
    @Test
    void stopMotor(LoopRunner runner) {
        runner.motor("belt_indexer");
        BeltIndexer beltIndexer = new BeltIndexer(runner.hardwareMap(),null);

        // action
        beltIndexer.hold().schedule();
        // time to run
        runner.loops(10);

        //verification
        assertEquals(0, beltIndexer.getIndexerPower(), 0.01);
    }
}
