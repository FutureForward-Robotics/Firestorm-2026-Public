package org.firstinspires.ftc.teamcode.subsystems.indexers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.BusWrites;

public class BeltIndexer extends Indexer{
    private DcMotorEx indexer;
    private Telemetry telemetry;
    private double lastWritten = 0;
    public BeltIndexer(HardwareMap hardwareMap, Telemetry telemetry) {
        indexer = hardwareMap.get(DcMotorEx.class, "belt_indexer");
        this.telemetry = telemetry;

    }


    @Override
    protected void onHold() {

        if (BusWrites.worthMotor(0, lastWritten)) {
            indexer.setPower(0);
            lastWritten = 0;
        }
    }

    @Override
    protected void onFeed() {
        if (BusWrites.worthMotor(1, lastWritten)) {
            indexer.setPower(1);
            lastWritten = 1;
        }
    }

    @Override
    protected void onUnjam() {
        if (BusWrites.worthMotor(-1, lastWritten)) {
            indexer.setPower(-1);
            lastWritten = -1;
        }
    }

    public double getIndexerPower() {
        return indexer.getPower();
    }

    @Override
    public void sense() {
    }
}
