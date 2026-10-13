package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * {@link VisionCamera} over a Limelight 3A: crosshair offsets and the fiducials in the frame, each
 * with its position in robot space.
 *
 * <p>The Limelight polls on its own schedule, so a reading is whatever frame finished last;
 * {@link VisionSample#stalenessMs()} is how old it was. A read never throws: a camera that is
 * unplugged mid-match yields {@link VisionSample#none()} rather than ending the OpMode.
 */
public final class LimelightCamera implements VisionCamera {

    /** The Limelight's own capture rate. 100 Hz is its maximum and twice the control loop. */
    public static final int DEFAULT_POLL_RATE_HZ = 100;

    private final Limelight3A limelight;

    public LimelightCamera(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, DEFAULT_POLL_RATE_HZ);
    }

    public LimelightCamera(HardwareMap hardwareMap, String name, int pollRateHz) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.setPollRateHz(pollRateHz);
        limelight.start();
    }

    @Override
    public void pipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    @Override
    public VisionSample read() {
        LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (RuntimeException e) {
            return VisionSample.none();
        }
        if (result == null || !result.isValid()) {
            return VisionSample.none();
        }

        VisionSample sample =
                new VisionSample(
                        true,
                        result.getStaleness(),
                        result.getPipelineIndex(),
                        result.getTx(),
                        result.getTy(),
                        result.getTa());

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            List<VisionSample.Tag> tags = new ArrayList<>(fiducials.size());
            for (int i = 0; i < fiducials.size(); i++) {
                LLResultTypes.FiducialResult f = fiducials.get(i);
                // Never null: the SDK fills a missing pose with zeros. Reported in meters.
                Position robotSpace =
                        f.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
                tags.add(
                        new VisionSample.Tag(
                                f.getFiducialId(),
                                f.getTargetXDegrees(),
                                f.getTargetYDegrees(),
                                f.getTargetArea(),
                                robotSpace.x,
                                robotSpace.y,
                                robotSpace.z));
            }
            sample = sample.withTags(tags);
        }
        return sample;
    }
}
