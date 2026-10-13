package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.util.BusWrites;
import org.firstinspires.ftc.teamcode.util.RunLog;

public class Intake extends ForwardSubsystem {

    public enum State {
        INTAKE,
        OFF,
        UNJAM
    }

    //Constants
    public static final double INTAKE_POWER = 1;

    private final DcMotorEx intake;
    private double lastWritten = 0;

    private State currentState = State.OFF;

    public Intake(DcMotorEx intakeMotor) {
        intake = intakeMotor;
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void sense() {
    }

    @Override
    public void logSignals(RunLog log) {
        log.addSignal("intake.state", () -> currentState.ordinal());
        log.addSignal("intake.power", () -> lastWritten);
    }

    @Override
    public void act() {
        switch (currentState) {
            case OFF:
                write(0);
                break;
            case INTAKE:
                write(INTAKE_POWER);
                break;
            case UNJAM:
                write(-INTAKE_POWER);
                break;
        }
    }

    private void write(double power) {
        if (BusWrites.worthMotor(power, lastWritten)) {
            intake.setPower(power);
            lastWritten = power;
        }
    }

    public Command stop() {
        return new InstantCommand(() -> currentState = State.OFF);
    }

    public Command start() {
        return new InstantCommand(() -> currentState = State.INTAKE);
    }

    public Command unjam() {
        return new InstantCommand(() -> currentState = State.UNJAM);
    }
}
