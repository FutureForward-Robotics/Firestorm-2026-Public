package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.baseautos.AutoTilt2;

@Autonomous(group = "A", name = "Blue 2 Tilt")
public class BlueAutoTilt2 extends AutoTilt2 {

    @Override
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
