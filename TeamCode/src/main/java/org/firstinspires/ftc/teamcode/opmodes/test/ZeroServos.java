package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Utility(name = "Zero Servos")
public class ZeroServos extends OpMode {
    private Servo ramp;

    @Override
    public void init() {

        this.ramp = hardwareMap.get(Servo.class, Intake.RAMP_NAME);


    }

    @Override
    public void loop() {
        double gatePos = -gamepad1.left_stick_y;
        double rampPos = -gamepad1.right_stick_y;

        ramp.setPosition(rampPos);

        telemetry.addData("gate", gatePos);
        telemetry.addData("ramp", rampPos);
        telemetry.update();
    }


}
