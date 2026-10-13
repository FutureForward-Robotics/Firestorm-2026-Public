# Limelight

The Limelight 3A is a camera that runs its own vision processing and reports results to the Robot
Controller. This repository wraps it in `Vision`, which reads the camera once per loop.

The camera is used for aiming: it reports the angle to what it sees, and you turn until that angle is
zero. It is not used to find the robot's position on the field. The camera's MegaTag2 position
estimate needs every AprilTag at a fixed, known spot, and this season the tags are on the HIVE
CELLS, which move whenever the HIVE tips.

## Setup

1. Plug the Limelight into a USB port on the Control Hub. It appears as an ethernet device.
2. On the Robot Controller configuration screen, find the Limelight in the device list and name it
   `limelight`. The number shown under the name is the IP address the Limelight assigned to the
   Control Hub, not the Limelight's own address.
3. Open the Limelight's web interface from a computer on the same network, at
   `http://limelight.local:5801`.
4. Set up a pipeline. For AprilTags, use an AprilTag pipeline and note its index.

## OpMode setup

```java
@Override
protected void configure() {
    robot = new Robot(hardwareMap, alliance());
    vision = new Vision(new LimelightCamera(hardwareMap, "limelight"));
}
```

`Vision` is not part of `Robot`, because `hardwareMap.get` throws if the configuration has no device
named `limelight`. Keeping it in the OpModes that use it means a robot without a camera still runs.

Its signals land in the run log on their own. `Vision` is a subsystem, and `ForwardOpMode` registers
every subsystem's signals after `configure()` returns. Do not call `vision.logSignals(log)` from the
OpMode's `logSignals` as well: `RunLog` rejects the second registration with `duplicate signal name`,
which stops the OpMode at init.

To see what the camera sees, run the **Vision Check** OpMode in the `test` group. It shows
`hasTarget`, the IDs of the tags in the frame, and how old the frame is.

## Aiming

```java
double bearing = vision.txForTag(24);       // degrees, positive to the right
if (!Double.isNaN(bearing)) {
    turret.nudge(-bearing * KP);
}
```

| Method | Returns |
| --- | --- |
| `hasTarget()` | True when the camera saw something in a frame new enough to use |
| `tx()`, `ty()` | Angle from the crosshair to the target, degrees, NaN with no target |
| `sawTag(int id)` | True when that AprilTag is in the current frame |
| `txForTag(int id)` | Angle to one specific tag, NaN when it is not in the frame |
| `sample()` | The whole reading, including the tag list |
| `pipeline(int index)` | Switch pipelines |

Methods return NaN rather than zero when there is no target, because zero means "dead ahead" and
would send a mechanism to the wrong place.

`Vision` reads the camera one time per loop and answers every question from that reading. Do not call
`getLatestResult()` yourself. If you read the camera twice in a loop you can get two answers from
two different frames.

## Constants

| Constant | Default | When to change it |
| --- | --- | --- |
| `Vision.MAX_STALENESS_MS` | 100 | Rarely. A frame older than this is ignored. |
| `LimelightCamera.DEFAULT_POLL_RATE_HZ` | 100 | Rarely. This is the camera's maximum. |
