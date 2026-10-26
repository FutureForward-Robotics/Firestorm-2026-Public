package org.firstinspires.ftc.teamcode.fakes;

import org.firstinspires.ftc.teamcode.util.VisionCamera;
import org.firstinspires.ftc.teamcode.util.VisionSample;

/**
 * Camera whose reading the test sets. Counts reads, because reading once per loop is the contract
 * {@code Vision} exists to enforce.
 */
public final class FakeVisionCamera implements VisionCamera {

    private VisionSample next = VisionSample.none();
    private int reads;
    private int pipeline;

    public void set(VisionSample sample) {
        next = sample;
    }

    @Override
    public VisionSample read() {
        reads++;
        return next;
    }

    @Override
    public void pipeline(int index) {
        pipeline = index;
    }

    public int reads() {
        return reads;
    }

    public int pipelineIndex() {
        return pipeline;
    }
}
