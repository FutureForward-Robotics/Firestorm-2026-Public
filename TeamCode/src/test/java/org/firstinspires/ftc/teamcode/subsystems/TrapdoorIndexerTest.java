package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.firstinspires.ftc.teamcode.subsystems.indexers.BeltIndexer;
import org.firstinspires.ftc.teamcode.subsystems.indexers.TrapdoorIndexer;
import org.junit.jupiter.api.Test;

@RobotTest
public class TrapdoorIndexerTest {
    @Test
    void startServo(LoopRunner runner) {
        runner.servo("trapdoor_indexer");
        TrapdoorIndexer trapdoorIndexer = new TrapdoorIndexer(runner.hardwareMap(),null);

        // action
        trapdoorIndexer.feed().schedule();
        // time to run
        runner.loops(10);

        //verification
        assertEquals(1, trapdoorIndexer.getIndexerPosition(), 0.01);
    }
    @Test
    void stopServo(LoopRunner runner) {
        runner.servo("trapdoor_indexer");
        TrapdoorIndexer trapdoorIndexer = new TrapdoorIndexer(runner.hardwareMap(),null);

        // action
        trapdoorIndexer.hold().schedule();
        // time to run
        runner.loops(10);

        //verification
        assertEquals(0, trapdoorIndexer.getIndexerPosition(), 0.01);
    }
}
