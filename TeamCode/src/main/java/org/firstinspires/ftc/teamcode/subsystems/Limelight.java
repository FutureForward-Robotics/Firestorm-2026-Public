package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.util.BusWrites;

import java.util.List;

public class Limelight extends ForwardSubsystem {
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private double degreesToTarget;
    private double distanceFromTarget;

    private int[] redIDs = {31 , 32, 35, 36};
    private int[] blueIDs = {39, 40, 43, 44};
    private Alliance alliance;


    public Limelight(HardwareMap hardwareMap, Alliance alliance, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        limelight.setPollRateHz(100);
        limelight.start();
        this.alliance = alliance;

    }

    @Override
    public void sense() {
        limelight.pipelineSwitch(0);
        LLResult result = limelight.getLatestResult();
        long staleness = result.getStaleness();
        if (result.isValid() && staleness <= 100) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                double distanceFromTargetX;
                double distanceFromTargetY;
                double distanceFromTargetZ;
                double raw;
                if (alliance.isRed()) {
                    for(int i = 0; i < redIDs.length; i++) {
                        int desiredId = redIDs[i];
                        if (id == desiredId) {
                            degreesToTarget = fiducial.getTargetXDegrees();
                            distanceFromTargetX = fiducial.getTargetPoseCameraSpace().getPosition().x;
                            distanceFromTargetY = fiducial.getTargetPoseCameraSpace().getPosition().y;
                            distanceFromTargetZ = fiducial.getTargetPoseCameraSpace().getPosition().z;
                            raw = distanceFromTargetX * distanceFromTargetX + distanceFromTargetY * distanceFromTargetY + distanceFromTargetZ * distanceFromTargetZ;
                            distanceFromTarget = Math.sqrt(raw);
                        }
                    }
                }
                else{
                    for(int i = 0; i < blueIDs.length; i++) {
                        int desiredId = blueIDs[i];
                        if (id == desiredId) {
                            degreesToTarget = fiducial.getTargetXDegrees();
                            degreesToTarget = fiducial.getTargetXDegrees();
                            distanceFromTargetX = fiducial.getTargetPoseCameraSpace().getPosition().x;
                            distanceFromTargetY = fiducial.getTargetPoseCameraSpace().getPosition().y;
                            distanceFromTargetZ = fiducial.getTargetPoseCameraSpace().getPosition().z;
                            raw = distanceFromTargetX * distanceFromTargetX + distanceFromTargetY * distanceFromTargetY + distanceFromTargetZ * distanceFromTargetZ;
                            distanceFromTarget = Math.sqrt(raw);
                        }
                    }
                }
            }
        }
        else{
            degreesToTarget = 0;
            distanceFromTarget=10000000;
        }
    }

    @Override
    public void act() {

    }
}