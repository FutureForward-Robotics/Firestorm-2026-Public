package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.ScoringElement;
import org.firstinspires.ftc.teamcode.subsystems.Assists;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Driver button map. */
public final class DriverBindings {

    private DriverBindings() {}

    private static final double PRECISION_SCALE = 0.35;

    public static void configure(Robot robot, GamepadEx driver, GamepadEx operator) {
        // GamepadEx.getLeftY already negates the raw axis, so forward needs no minus; the X axes
        // are not pre-negated and do.
        robot.drive.setDefaultCommand(
                robot.drive.teleop(
                        driver::getLeftY,
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(robot.drive.assist(Assists.speedCap(PRECISION_SCALE)));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(robot.intake.start())
                .whenHeld(robot.indexer.load())
                .whenReleased(robot.intake.stop());

        driver.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(robot.nectarShooter.off())
                        .whenPressed(robot.pollenShooter.off());


        bindShooter(robot, driver, GamepadKeys.Button.RIGHT_BUMPER, ScoringElement.POLLEN);

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(robot.indexer.feed(ScoringElement.POLLEN))
                .whenInactive(robot.indexer.hold());

    }






    public static void bindShooter(Robot robot, GamepadEx gamepad,
                                   GamepadKeys.Button button, ScoringElement element) {

        Shooter shooter = robot.shooterFor(element);
        gamepad.getGamepadButton(button)
                .whenPressed(shooter.spinUp())
                .whenReleased(shooter.idle());
    }
}
