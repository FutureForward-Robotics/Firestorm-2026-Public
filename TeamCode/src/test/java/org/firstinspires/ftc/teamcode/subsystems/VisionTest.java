package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.fakes.FakeVisionCamera;
import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Hive;
import org.firstinspires.ftc.teamcode.util.RunLog;
import org.firstinspires.ftc.teamcode.util.VisionSample;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.List;

/**
 * Vision against a fake camera: one read per loop, staleness gating, tag lookup and finding the
 * upward CELL.
 */
@RobotTest
class VisionTest {

    private static final double EPS = 1e-9;

    /** Tag heights either side of the cutoff, inches. */
    private static final double UP_Z = Hive.UP_CUTOFF_IN + 10;
    private static final double DOWN_Z = Hive.UP_CUTOFF_IN - 10;

    /** Red's tags: 30 is on the FAR CELL, 34 on the AUDIENCE CELL. Blue's 42 is never ours. */
    private static final int RED_FAR_TAG = 30;
    private static final int RED_FAR_TAG_2 = 31;
    private static final int RED_AUDIENCE_TAG = 34;
    private static final int BLUE_FAR_TAG = 42;

    private static VisionSample fresh() {
        return new VisionSample(true, 20, 0, 3.5, -1.25, 4.0);
    }

    /** A fresh frame with only the given tags. */
    private static VisionSample seeing(VisionSample.Tag... tags) {
        return new VisionSample(true, 20, 0, 0, 0, 0).withTags(Arrays.asList(tags));
    }

    private static VisionSample.Tag tag(int id, double xIn, double yIn, double zIn) {
        return new VisionSample.Tag(id, 0, 0, 0, xIn, yIn, zIn);
    }

    private static void assertNoUpCell(Vision vision) {
        assertNull(vision.upCell());
        assertTrue(Double.isNaN(vision.bearingRad()), "no answer is NaN, not straight ahead");
        assertTrue(Double.isNaN(vision.heightIn()));
        assertTrue(Double.isNaN(vision.distanceIn()));
    }

    @Test
    void readsTheCameraOncePerLoopHoweverManyQuestionsAreAsked(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, Alliance.RED);
        camera.set(fresh());

        runner.loop();
        vision.hasTarget();
        vision.tx();
        vision.ty();
        vision.sawTag(21);

        assertEquals(1, camera.reads(), "four consumers, one network round-trip");
    }

    @Test
    void aStaleFrameIsNotATarget(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera,Alliance.RED);
        camera.set(new VisionSample(true, Vision.MAX_STALENESS_MS + 1, 0, 3.5, 0, 4.0));

        runner.loop();

        assertFalse(vision.hasTarget(), "valid but older than the staleness limit");
        assertTrue(Double.isNaN(vision.tx()), "no fresh target means no angle, not a stale one");
    }

    @Test
    void anInvalidFrameYieldsNoTarget(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera);
        camera.set(VisionSample.none());

        runner.loop();

        assertFalse(vision.hasTarget());
        assertTrue(Double.isNaN(vision.txForTag(24)));
    }

    @Test
    void findsTheAngleToOneTagAndIgnoresTheOthers(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera);
        camera.set(
                fresh().withTags(
                                Arrays.asList(
                                        new VisionSample.Tag(20, -7.5, 1.0, 2.0, 40, -6, 12),
                                        new VisionSample.Tag(24, 12.25, 2.0, 3.0, 36, 9, 14))));

        runner.loop();

        assertTrue(vision.sawTag(24));
        assertFalse(vision.sawTag(21));
        assertEquals(12.25, vision.txForTag(24), EPS);
        assertTrue(Double.isNaN(vision.txForTag(21)), "absent tag is NaN, not zero");
    }

    @Test
    void aTagOfOursAboveTheCutoffIsTheUpCell(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, Alliance.RED);

        camera.set(seeing(tag(RED_FAR_TAG, 40, 0, UP_Z)));
        runner.loop();
        assertEquals(Hive.Cell.FAR, vision.upCell());

        camera.set(seeing(tag(RED_FAR_TAG, 40, 0, DOWN_Z)));
        runner.loop();
        assertNoUpCell(vision);
    }

    @Test
    void theUpCellIsTheOneAboveTheCutoff(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, Alliance.RED);
        camera.set(
                seeing(tag(RED_FAR_TAG, 40, 0, DOWN_Z), tag(RED_AUDIENCE_TAG, 40, 20, UP_Z)));

        runner.loop();

        assertEquals(Hive.Cell.AUDIENCE, vision.upCell());
        assertEquals(UP_Z, vision.heightIn(), EPS);
    }

    @Test
    void theOtherAlliancesTagsAreIgnored(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, Alliance.RED);
        camera.set(seeing(tag(BLUE_FAR_TAG, 40, 0, UP_Z)));

        runner.loop();

        assertNoUpCell(vision);
    }

    @Test
    void aStaleFrameHasNoUpCell(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, Alliance.RED);
        camera.set(
                new VisionSample(true, Vision.MAX_STALENESS_MS + 1, 0, 0, 0, 0)
                        .withTags(Arrays.asList(tag(RED_FAR_TAG, 40, 0, UP_Z))));

        runner.loop();

        assertNoUpCell(vision);
    }

    @Test
    void aCellAheadAndToTheLeftHasAPositiveBearing(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, Alliance.RED);
        // Y points right, so a negative y is to the robot's left.
        camera.set(seeing(tag(RED_FAR_TAG, 40, -10, UP_Z)));

        runner.loop();

        assertTrue(vision.bearingRad() > 0, "counter-clockwise, toward the left");
    }

    @Test
    void distanceAndHeightAverageTheCellsTags(LoopRunner runner) {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera, Alliance.RED);
        camera.set(seeing(tag(RED_FAR_TAG, 40, -10, 52), tag(RED_FAR_TAG_2, 48, -2, 54)));

        runner.loop();

        // Averages: x 44, y -6, z 53. Distance is horizontal only: sqrt(44^2 + 6^2).
        assertEquals(Hive.Cell.FAR, vision.upCell());
        assertEquals(53, vision.heightIn(), EPS);
        assertEquals(44.4072066, vision.distanceIn(), 1e-6);
    }

    @Test
    void signalsSurviveAnEmptyFrame(LoopRunner runner, @TempDir File logDir) throws IOException {
        FakeVisionCamera camera = new FakeVisionCamera();
        Vision vision = new Vision(camera);
        camera.set(VisionSample.none());

        RunLog log = RunLog.open(logDir, "Vision");
        vision.logSignals(log);
        runner.loop();
        log.writeLoop(0);
        log.close();

        List<String> csv = Files.readAllLines(new File(log.directory(), "signals.csv").toPath());
        assertEquals(
                "t_s,loop,vision.hasTarget,vision.stalenessMs,vision.tx,vision.ty,vision.tags,"
                        + "vision.upCell,vision.bearingRad,vision.heightIn,vision.distanceIn",
                csv.get(0));
        // No target means an empty field, not an NPE swallowed by writeLoop's supplier guard.
        String[] fields = csv.get(1).split(",", -1);
        assertEquals("0", fields[2], "hasTarget is false");
        assertEquals("", fields[4], "tx is NaN with no target");
        assertEquals("-1.0000", fields[7], "upCell is -1 when unknown");
        assertEquals("", fields[8], "bearingRad is NaN when unknown");
        assertEquals(0, log.dropped());
    }
}
