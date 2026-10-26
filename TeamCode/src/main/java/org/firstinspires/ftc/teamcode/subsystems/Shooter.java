package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.BusWrites;
import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.function.DoubleSupplier;

public class Shooter extends ForwardSubsystem {

    public enum State {
        OFF,
        IDLE,
        SHOOT
    }

    private final DcMotorEx shooterMotor;
    private DoubleSupplier targetSupplier = () -> 0;
    private double targetRpm = 0;
    private double currentRpm;
    private double lastWritten = 0;
    private double kP;
    private double error;
    private String name;

    //Constants
    public static final double IDLE_RPM = 2000;
    public static final double SHOOT_RPM = 2100;
    public static final double FIRST_AUTO_SHOOT = 2175;
    public static final double RPM_TOLERANCE = 50;
    public static final double TICKS_PER_REV = 28;
    public static final double MAX_RPM = 4900;
    public double offset = 0;



    private State currentState = State.OFF;

    public Shooter(String name, DcMotorEx shooterMotor, double kP) {
        this.shooterMotor = shooterMotor;
        this.shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.kP = kP;
        this.name = name;
    }



    public double clamp(double pow){
        if(pow>1){
            return 1;
        }
        else if(pow<0){
            return -0.05;
        }
        return pow;
    }

    @Override
    public void sense() {
        targetRpm = targetSupplier.getAsDouble();
        currentRpm = shooterMotor.getVelocity()/TICKS_PER_REV*60;
        error = setpointRpm() - currentRpm;
    }

    @Override
    public void act() {
        double power = setpointRpm() / MAX_RPM + error * kP;
        double heldPower = clamp(power);
        if (currentState == State.SHOOT || currentState == State.IDLE) {
            if (BusWrites.worthMotor(heldPower, lastWritten)) {
                shooterMotor.setPower(heldPower);
                lastWritten = heldPower;
            }
        } else {
            if (BusWrites.worthMotor(0, lastWritten)) {
                shooterMotor.setPower(0);
                lastWritten = 0;
            }
        }
    }

    @Override
    public void logSignals(RunLog log) {
        log.addSignal(name + ".state", () -> currentState.ordinal());
        log.addSignal(name + ".rpm", () -> currentRpm);
        log.addSignal(name + ".targetRpm", () -> targetRpm);
        log.addSignal(name + ".power", () -> lastWritten);
        log.addSignal(name + ".rpmOffset",()-> offset);
        log.addFlag(name + ".atSpeed",  this::atSpeed);
        log.addSlowSignal(name + ".currentA", () -> shooterMotor.getCurrent(CurrentUnit.AMPS), 100);
    }


    public Command off() {
        return new InstantCommand(() -> currentState = State.OFF);
    }

    public Command offsetUp() {
        return new InstantCommand(() -> offset+=25);
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public double getTargetRpm() {
        return setpointRpm();
    }

    // The D-pad offset trims shots only, not the idle speed.
    private double setpointRpm() {
        return currentState == State.SHOOT ? targetRpm + offset : targetRpm;
    }


    public Command offsetDown(){
        return new InstantCommand(()-> offset -=25);
    }

    public Command idle() {
        return new InstantCommand(() -> {
            track(() -> IDLE_RPM);
            currentState = State.IDLE;
        });
    }

    public Command spinUp() {
        return new InstantCommand(() -> {
            track(() -> SHOOT_RPM);
            currentState = State.SHOOT;
        });
    }
    /** Follows {@code rpm} every loop until another command changes the target. */
    public Command spinUp(DoubleSupplier rpm) {
        return new InstantCommand(() -> {
            track(rpm);
            currentState = State.SHOOT;
        });
    }

    // Reads the target now as well, so act() in this loop already uses it.
    private void track(DoubleSupplier rpm) {
        targetSupplier = rpm;
        targetRpm = rpm.getAsDouble();
    }

    public boolean atSpeed() {
        return Math.abs(setpointRpm() - currentRpm) < RPM_TOLERANCE && currentState == State.SHOOT;
    }

    public Command waitForSpeed() {
        return new WaitUntilCommand(this::atSpeed);
    }
}