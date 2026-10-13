package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.fakes.FakeMotor;
import org.firstinspires.ftc.teamcode.fakes.LoopRunner;
import org.firstinspires.ftc.teamcode.fakes.RobotTest;
import org.junit.jupiter.api.Test;

@RobotTest
public class IntakeTest {

    private static final double EPS = 1e-9;

    @Test
    void startKeepsIntakePowerEveryLoop(LoopRunner runner) {
        //a fake motor named "intake", and the real Intake built on it.
        FakeMotor motor = runner.motor("intake");
        Intake intake = new Intake(motor);

        //do what the driver's button does, then run the robot loop 5 times.
        intake.start().schedule();
        runner.loops(5);

        assertEquals(Intake.INTAKE_POWER, motor.getPower(), EPS);
    }

    @Test
    void onStartPowerIsZero(LoopRunner runner) {
        //a fake motor named "intake", and the real Intake built on it.
        FakeMotor motor = runner.motor("intake");
        Intake intake = new Intake(motor);

        //no command scheduled: run the robot loop once.
        runner.loops(1);

        assertEquals(0, motor.getPower(), EPS);
    }

    @Test
    void unjamReversesIntake(LoopRunner runner) {
        //a fake motor named "intake", and the real Intake built on it.
        FakeMotor motor = runner.motor("intake");
        Intake intake = new Intake(motor);

        //schedule unjam, then run the robot loop 5 times.
        intake.unjam().schedule();
        runner.loops(5);
        assertEquals(-Intake.INTAKE_POWER, motor.getPower(), EPS);
    }

    @Test
    void intakeCanStop(LoopRunner runner) {
        FakeMotor motor = runner.motor("intake");
        Intake intake = new Intake(motor);

        //start the intake, run 5 loops, then stop it and run 5 more.
        intake.start().schedule();
        runner.loops(5);

        intake.stop().schedule();
        runner.loops(5);

        assertEquals(0, motor.getPower(), EPS);
    }
}
