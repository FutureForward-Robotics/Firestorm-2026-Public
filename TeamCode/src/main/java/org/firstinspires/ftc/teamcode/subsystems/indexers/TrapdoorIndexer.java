package org.firstinspires.ftc.teamcode.subsystems.indexers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.BusWrites;

public class TrapdoorIndexer extends Indexer{
    private Servo indexer;
    private Telemetry telemetry;
    private int lastWritten = 0;
    public TrapdoorIndexer(HardwareMap hardwareMap, Telemetry telemetry) {
        indexer = hardwareMap.get(Servo.class, "trapdoor_indexer");
        this.telemetry = telemetry;

    }


    @Override
    protected void onHold() {

        if (BusWrites.worthServo(0, lastWritten)) {
            indexer.setPosition(0);
            lastWritten = 0;
        }
    }

    @Override
    protected void onFeed() {
        if (BusWrites.worthServo(1, lastWritten)) {
            indexer.setPosition(1);
            lastWritten = 1;
        }
    }

    @Override
    protected void onUnjam() {

    }

    @Override
    public void sense() {
    }
    public double getIndexerPosition(){
        return indexer.getPosition();
    }
}
