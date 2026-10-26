package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

    private boolean rampDeployed = false;
    private boolean sideRollerActive = false;

    //Constants
    public static final double INTAKE_POWER = 1;
    public static final String MOTOR_NAME = "intake";
    public static final String RAMP_NAME = "ramp";
    public static final String SIDE_ROLLER_NAME = "sideRoller";

    public static final double RAMP_STOWED = 0.9;
    private final DcMotorEx intake;
    private final Servo ramp;
    private final CRServo sideRoller;
    private double lastWritten = 0;
    private double lastRampPos = 0;
    private double lastSideRoller = 0;

    private State currentState = State.OFF;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        ramp = hardwareMap.get(Servo.class, RAMP_NAME);
        sideRoller = hardwareMap.get(CRServo.class, SIDE_ROLLER_NAME);
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
                sideRollerActive = false;
                break;
            case INTAKE:
                write(INTAKE_POWER);
                sideRollerActive = true;
                break;
            case UNJAM:
                write(-INTAKE_POWER);
                break;
        }

        double pos = rampDeployed ? 0.1 : RAMP_STOWED;
        double sideRollerPow = sideRollerActive ? -1 : 0;
        if (BusWrites.worthServo(pos, lastRampPos)) {
            ramp.setPosition(pos);
            lastRampPos = pos;
        }

        if (BusWrites.worthServo(sideRollerPow, lastSideRoller)) {
            sideRoller.setPower(sideRollerPow);
            lastSideRoller = sideRollerPow;
        }
    }

    private void write(double power) {
        if (BusWrites.worthMotor(power, lastWritten)) {
            intake.setPower(power);
            lastWritten = power;
        }
    }

    public Command rampDown(){
        return new InstantCommand(()-> ramp.setPosition(0.02));
    }

    public Command rampUp(){
        return new InstantCommand(()-> ramp.setPosition(0.9));
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
