package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.BusWrites;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Shooter extends ForwardSubsystem {

    public enum State {
        HOLD,
        OFF
    }

    private final DcMotorEx shooter;
    private double rpm = 0;
    private double target_rpm=3000;
    private double current_rpm;
    private double max_rpm=5000;
    private double lastWritten = 0;
    private double kP;

    private State currentState = State.OFF;
    public Shooter(DcMotorEx shooterMotor, double kP) {
        shooter = shooterMotor;
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.kP = 0.02;
    }



    @Override
    public void sense() {
        current_rpm = shooter.getVelocity();
    }

    @Override
    public void act() {

        switch (currentState) {
            case OFF:
                if (BusWrites.worthMotor(0, lastWritten)) {
                    shooter.setPower(0);
                    lastWritten = 0;
                }
                break;
            case HOLD:
                double error = (rpm - current_rpm)*kP;
                if (rpm==0){
                    error = 0;
                }
                double power = rpm/max_rpm* + error;

                if (BusWrites.worthMotor(power, lastWritten)) {
                    shooter.setPower(power);
                    lastWritten = power;
                }
                break;
        }
    }

    public Command disableShooter() {
        return new InstantCommand(() -> currentState = State.OFF);
    }

    public Command readyShooter() {
        return new InstantCommand(() -> currentState = State.HOLD);
    }

    public boolean atRPM() {
        return target_rpm - 100 < current_rpm && current_rpm < target_rpm + 100;
    }

    public Command atRPMCmd() {
        return new WaitUntilCommand(this::atRPM);
    }
}