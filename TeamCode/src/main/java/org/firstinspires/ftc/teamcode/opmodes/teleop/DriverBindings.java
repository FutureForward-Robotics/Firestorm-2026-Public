package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.ScoringElement;
import org.firstinspires.ftc.teamcode.field.Waypoints;
import org.firstinspires.ftc.teamcode.subsystems.Assists;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Driver button map. */
public final class DriverBindings {

    private DriverBindings() {}

    private static final double PRECISION_SCALE = 0.35;
    private static final double AIMED_TOLERANCE_RAD = Math.toRadians(1);
    private static final boolean READY_NEEDS_SHOOTER_SPEED = false;

    public static void configure(Robot robot, GamepadEx driver, GamepadEx operator) {
        // GamepadEx.getLeftY already negates the raw axis, so forward needs no minus; the X axes
        // are not pre-negated and do. Field-centric forward is +x; the blue drivers face the other way.
        double allianceSign = robot.alliance.isRed() ? 1 : -1;
        robot.drive.setDefaultCommand(
                robot.drive.teleop(
                        () -> allianceSign * driver.getLeftY(),
                        () -> -allianceSign * driver.getLeftX(),
                        () -> -driver.getRightX()));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(robot.drive.assist(Assists.speedCap(PRECISION_SCALE)));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(robot.intake.start())
                .whenHeld(robot.indexer.load())
                .whenReleased(robot.intake.stop())
                .whenReleased(robot.indexer.hold());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(robot.intake.unjam())
                .whenReleased(robot.intake.stop());

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(robot.pollenShooter.offsetUp());

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(robot.pollenShooter.offsetDown());

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(robot.indexer.feed(ScoringElement.POLLEN))
                .whenInactive(robot.indexer.hold());

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenActive(robot.intake.rampDown());

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenActive(robot.intake.rampUp());

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> robot.drive.setPose(Waypoints.bottomAutoStart.pose(robot.alliance)));

        Trigger pollenAimHeld =
                new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);

        pollenAimHeld
                .whenActive(robot.pollenShooter.spinUp(robot.pollenCalculator::wheelRpm))
                .whileActiveContinuous(robot.drive.assist(Assists.headingLock(
                        robot.pollenCalculator::robotHeading, 1.05, 0.06)))
                .whenInactive(robot.pollenShooter.idle());

        pollenAimHeld
                .and(new Trigger(() -> pollenShotIsReady(robot)))
                .whenActive(() -> driver.gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS))
                .whenInactive(driver.gamepad::stopRumble);




    }

    private static boolean pollenShotIsReady(Robot robot) {
        boolean isAimed = Math.abs(robot.pollenCalculator.headingError()) < AIMED_TOLERANCE_RAD;
        boolean isAtSpeed = !READY_NEEDS_SHOOTER_SPEED || robot.pollenShooter.atSpeed();
        return robot.pollenCalculator.canShoot() && isAimed && isAtSpeed;
    }
}
