package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.baseautos.PathTestPartnerTop;
import org.firstinspires.ftc.teamcode.opmodes.auto.baseautos.PathTestTilt2;

@Autonomous(group = "C", name = "Red Path Test")
public class RedPathTest extends PathTestPartnerTop {

    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
