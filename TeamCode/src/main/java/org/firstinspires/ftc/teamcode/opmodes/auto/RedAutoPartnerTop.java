package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.baseautos.AutoPartnerTop;
import org.firstinspires.ftc.teamcode.opmodes.auto.baseautos.AutoTilt2;

@Autonomous(group = "B", name = "Red Auto Partner Top")
public class RedAutoPartnerTop extends AutoPartnerTop {

    @Override
    public Alliance getAlliance() {
        return Alliance.RED;
    }
}
