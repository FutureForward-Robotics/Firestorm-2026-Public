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

public class Intake extends ForwardSubsystem {



    public enum State {
        INTAKE,
        OFF,
        UNJAM
    }

    private final DcMotorEx intake;
    private double rpm = 0;
    private double target_rpm=3000;
    private double current_rpm;
    private double max_rpm=5000;
    private double lastWritten = 0;
    private double kP;

    private State currentState = State.OFF;
    public Intake(DcMotorEx intakeMotor) {
        intake = intakeMotor;
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void sense() {

    }

    @Override
    public void act() {
        switch (currentState) {
            case OFF:
                if (BusWrites.worthMotor(0, lastWritten)) {
                    intake.setPower(0);
                    lastWritten = 0;
                }
                break;
            case INTAKE:
                if (BusWrites.worthMotor(1, lastWritten)) {
                    intake.setPower(1);
                    lastWritten = 1;
                }

                if (BusWrites.worthMotor(-1, lastWritten)) {
                    intake.setPower(-1);
                    lastWritten = -1;
                }
                break;
            }
        }

        public Command stopIntakeCmd() {
            return new InstantCommand(() -> currentState = State.OFF);
        }

        public Command intakeCmd() {
            return new InstantCommand(() -> currentState = State.INTAKE);
        }
        public Command outtakeCmd() {
            return new InstantCommand(() -> currentState = State.UNJAM);
        }

    }




