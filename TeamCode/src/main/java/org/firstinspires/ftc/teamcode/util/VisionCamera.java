package org.firstinspires.ftc.teamcode.util;

/**
 * Read side of a vision camera, so everything above it is testable without hardware. */
public interface VisionCamera {

    /** One reading. Never null; {@link VisionSample#none()} when nothing was seen. */
    VisionSample read();

    void pipeline(int index);
}
