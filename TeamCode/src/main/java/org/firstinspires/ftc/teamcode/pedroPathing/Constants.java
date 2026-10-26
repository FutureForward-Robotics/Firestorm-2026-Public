package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing configuration: a mecanum drivetrain, a Pinpoint localizer, and the Foresight
 * following algorithm.
 *
 * <p>Every number here comes from AutoTune. Run the tuners from the Driver Station, open the page
 * it hosts, and paste the block each one prints over the matching config below. The values checked
 * in are placeholders that let the code compile; they do not describe any real robot.
 *
 * <p>Foresight requires the velocity, deceleration and braking values: it plans a braking point
 * from them rather than easing toward the end, so a wrong number shows up as overshoot.
 */
public class Constants {

    public static final MecanumConfig drivetrainConfig =
            new MecanumConfig(
                    c -> {
                        c.frontLeftName.set("leftFront");
                        c.frontRightName.set("rightFront");
                        c.backLeftName.set("leftBack");
                        c.backRightName.set("rightBack");
                        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);//sus
                        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);//sus
                        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                    });

    public static final PinpointConfig localizerConfig =
            new PinpointConfig(
                    c -> {
                        c.name.set("pinpoint");
                        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                        c.xPodOffset.set(6.960374126284141);
                        c.yPodOffset.set(0.5389996025505968);
                        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                        c.globalDistanceUnit.set(DistanceUnit.INCH);
                        c.offsetUnits.set(DistanceUnit.INCH);
                    });

    public static final ForesightConfig algorithmConfig =
            new ForesightConfig(
                    c -> {
                        Controller primaryTranslationalForward = Controller.proportional(0.3463164754158047);
                        Controller secondaryTranslationalForward = Controller.proportional(0.12795468391948803);
                        Controller primaryTranslationalLateral = Controller.proportional(.35);
                        Controller secondaryTranslationalLateral = Controller.proportional(.13);

                        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                        c.coast.set(Controller.proportionalFeedforward(0.012629642625427557));
                        c.brake.set(Controller.proportionalFeedforward(0.010735196231613424));

                        c.headingFeedback.set(Controller.proportional(2));
                        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.02));

                        c.linearBrakeCoefficients.set(Matrix.diag(0.12342997577090055, -0.010831544604387245));
                        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011994325047654957, 0.0036506977594995146));

                        c.maxAchievableForwardVelocity.set(77.96792663345977);
                        c.maxAchievableStrafeVelocity.set(59.10610864565133);
                        c.naturalForwardDeceleration.set(27.67771188022932);
                        c.naturalStrafeDeceleration.set(64.81400282599348);
                    });

    /** Hardware factories, separate so the AutoTune procedures can build them on their own. */
    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Algorithm algorithm() {
        return new Foresight(algorithmConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), algorithm());
    }
}
