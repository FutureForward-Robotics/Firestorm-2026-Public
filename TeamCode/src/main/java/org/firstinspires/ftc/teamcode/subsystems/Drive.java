package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.Waypoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.MotionSource;
import org.firstinspires.ftc.teamcode.util.PoseStore;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Drivetrain wrapping Pedro's {@link Follower}.
 *
 * <p>Motion commands require this subsystem, stop the follower when interrupted, and carry a
 * timeout. {@link #teleop} is the default command; a path command preempts it and the
 * scheduler restores it when the path ends.
 */
public class Drive extends ForwardSubsystem implements MotionSource {

    /** Path watchdog, milliseconds. */
    public static final long DEFAULT_PATH_TIMEOUT_MS = 8000;

    public static final long DEFAULT_TURN_TIMEOUT_MS = 3000;

    /** A turn command finishes inside this heading error, in radians. */
    public static final double TURN_TOLERANCE_RAD = Math.toRadians(2);

    private final Follower follower;
    private final List<DriveAssist> assists = new ArrayList<>();

    // Last good reading; use this, not follower.pose().
    private Pose trustedPose;

    // What teleop last asked for, kept for the run log. Stale while a path is being followed.
    private double driverTurn;
    private DriveInput commandedInput = new DriveInput(0, 0, 0);

    public Drive(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);
        trustedPose = startingPose;
    }

    /** Resumes from the stored pose. Use in teleop, after auto. */
    public Drive(HardwareMap hardwareMap) {
        this(hardwareMap, PoseStore.loadOr(Pose.zero()));
    }

    /**
     * Pedro's {@code update()} refreshes localization and writes motor powers in one call and
     * cannot be split. {@code updatePose()} must not be called separately: {@code
     * PoseTracker.update()} shifts {@code previousPoseTime}, so a second call in the same loop
     * measures velocity over roughly zero elapsed time.
     *
     * <p>Drive vectors set by a command during this loop are applied by the next loop's update.
     */
    @Override
    public void sense() {
        follower.update();
        Pose reading = follower.pose();
        boolean interrupted = Thread.currentThread().isInterrupted();
        if (!isTrustworthyReading(reading, follower.velocity(), trustedPose, interrupted)) {
            return;
        }
        trustedPose = reading;
        PoseStore.save(reading);
    }

    // A stop during a Pinpoint read returns all zeros; don't log or store them as the robot's pose.
    static boolean isTrustworthyReading(
            Pose reading, Velocity velocity, Pose lastTrusted, boolean threadInterrupted) {
        if (threadInterrupted) {
            return false;
        }
        boolean readingIsAllZero = isOrigin(reading) && isStill(velocity);
        return !readingIsAllZero || isOrigin(lastTrusted);
    }

    private static boolean isOrigin(Pose pose) {
        return pose.x() == 0 && pose.y() == 0 && pose.heading() == 0;
    }

    private static boolean isStill(Velocity velocity) {
        return velocity.vx == 0 && velocity.vy == 0 && velocity.omega == 0;
    }

    /** Empty; the follower wrote motor powers during {@link #sense()}. */
    @Override
    public void act() {
    }

    /** Reads the pose cached by {@link #sense()}, so logging adds no hardware traffic. */
    @Override
    public void logSignals(RunLog log) {
        log.addSignal("drive.x", () -> trustedPose.x());
        log.addSignal("drive.y", () -> trustedPose.y());
        log.addSignal("drive.headingDeg", () -> Math.toDegrees(trustedPose.heading()));
        log.addFlag("drive.busy", follower::isBusy);
        log.addSignal("drive.assists", () -> assists.size());
        log.addSignal("drive.vx", () -> follower.velocity().vx);
        log.addSignal("drive.vy", () -> follower.velocity().vy);
        log.addSignal("drive.driverTurn", () -> driverTurn);
        log.addSignal("drive.cmdForward", () -> commandedInput.forward);
        log.addSignal("drive.cmdStrafe", () -> commandedInput.strafe);
        log.addSignal("drive.cmdTurn", () -> commandedInput.turn);
    }

    // ---------------------------------------------------------------- state

    @Override
    public Pose pose() {
        return trustedPose;
    }

    /** Field-frame velocity, inches/second, from the pose cached by {@link #sense()}. */
    @Override
    public double velocityX() {
        return follower.velocity().vx;
    }

    @Override
    public double velocityY() {
        return follower.velocity().vy;
    }

    /** Re-seeds odometry, for squaring up on a wall mid-match. */
    public void setPose(Pose pose) {
        follower.setPose(pose);
        trustedPose = pose;
    }

    public double headingRad() {
        return trustedPose.heading();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public List<DriveAssist> activeAssists() {
        return assists;
    }

    /** Starts a {@link Route} on this drivetrain. */
    public Route route(Alliance alliance, Waypoint start) {
        return Route.from(alliance, start);
    }

    // ---------------------------------------------------------------- commands

    /**
     * Default command. Stick input passes through the assist stack. Suppliers must already be in
     * {@link DriveInput}'s sign convention. Field-centric.
     */
    public Command teleop(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return teleop(forward, strafe, turn, false);
    }

    public Command teleop(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, boolean robotCentric) {
        return new FunctionalCommand(
                () -> follower.manual(DrivePowers.zero()),
                () -> {
                    DriveInput input =
                            new DriveInput(
                                    forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
                    for (int i = 0; i < assists.size(); i++) {
                        input = assists.get(i).apply(input, trustedPose);
                    }
                    driverTurn = turn.getAsDouble();
                    commandedInput = input;
                    DrivePowers powers = new DrivePowers(input.forward, input.strafe, input.turn);
                    follower.manual(
                            robotCentric
                                    ? powers
                                    : ManualDrive.fieldCentric(powers, trustedPose.heading()));
                },
                interrupted -> {},
                () -> false,
                this);
    }

    public Command follow(Path path) {
        return follow(path, DEFAULT_PATH_TIMEOUT_MS);
    }

    /**
     * Follows a path, stopping the follower if the command is interrupted.
     *
     * <p>To cap one leg's speed, modify the path rather than this command: {@code
     * path.with(Constants.algorithmConfig.maxPathSpeed.at(0.5))} holds it to half the robot's top
     * speed and restores the previous limit when the path ends. {@code maxPathSpeed} is a fraction;
     * for an absolute limit use {@code maxVelocityConstraint}, in inches/second.
     */
    public Command follow(Path path, long timeoutMs) {
        return new FunctionalCommand(
                        () -> follower.follow(path),
                        () -> {},
                        interrupted -> {
                            if (interrupted) {
                                follower.stop();
                            }
                        },
                        () -> !follower.isBusy(),
                        this)
                .withTimeout(timeoutMs);
    }

    public Command turnTo(double headingDeg) {
        return turnTo(headingDeg, DEFAULT_TURN_TIMEOUT_MS);
    }

    /**
     * Turns in place by holding the current position with a new heading. Finishes inside {@link
     * #TURN_TOLERANCE_RAD}, since holding a pose never completes on its own.
     */
    public Command turnTo(double headingDeg, long timeoutMs) {
        double target = Math.toRadians(headingDeg);
        return new FunctionalCommand(
                        () -> follower.hold(pose().withHeading(target)),
                        () -> {},
                        interrupted -> follower.stop(),
                        () ->
                                Math.abs(Field.normalize(target - trustedPose.heading()))
                                        < TURN_TOLERANCE_RAD,
                        this)
                .withTimeout(timeoutMs);
    }

    /** Holds a pose. Never finishes on its own; race it or let the driver take the drivetrain. */
    public Command holdAt(Waypoint waypoint, Alliance alliance) {
        return new FunctionalCommand(
                () -> follower.hold(waypoint.pose(alliance)),
                () -> {},
                interrupted -> follower.stop(),
                () -> false,
                this);
    }

    /** Stops following. */
    public Command halt() {
        return new FunctionalCommand(
                follower::stop, () -> {}, interrupted -> {}, () -> true, this);
    }

    /**
     * Enables an assist while this command runs. Requires no subsystems. Bind with {@code
     * toggleWhenPressed} for a latch or {@code whenHeld} for hold-to-use.
     */
    public Command assist(DriveAssist assist) {
        return new StartEndCommand(
                () -> {
                    if (!assists.contains(assist)) {
                        assists.add(assist);
                    }
                },
                () -> assists.remove(assist));
    }

    /**
     * Arms a shot on the move: holds the heading the robot had when this was scheduled and drives
     * at a constant speed in whatever direction the sticks ask for.
     *
     * <p>Requires no subsystems, so it layers over the teleop default command. Bind it with {@code
     * whenHeld}: {@code whileHeld} reschedules every loop, which would re-latch the heading
     * continuously and hold nothing.
     */
    public Command steadyShot(double speed, double headingKp) {
        double[] heldHeading = new double[1];
        DriveAssist assist = Assists.steadyShot(() -> heldHeading[0], headingKp, speed);
        return new StartEndCommand(
                () -> {
                    heldHeading[0] = pose().heading();
                    if (!assists.contains(assist)) {
                        assists.add(assist);
                    }
                },
                () -> assists.remove(assist));
    }
}
