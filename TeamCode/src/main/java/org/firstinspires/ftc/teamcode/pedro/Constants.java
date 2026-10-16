package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
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

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontleft");
        c.frontRightName.set("frontright");
        c.backLeftName.set("backleft");
        c.backRightName.set("backright");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.7654831052765134);
        c.yPodOffset.set(-4.60075318343996);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4717722632991266);
                Controller secondaryTranslationalForward = Controller.proportional(0.17430724530198413);
                Controller primaryTranslationalLateral = Controller.proportional(0.4732554905206718);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1748552581277838);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01655423337620932));
                c.brake.set(Controller.proportionalFeedforward(0.01407109836977792));

                c.headingFeedback.set(Controller.proportional(10.224771820871675));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03683075413152864, 0.004667278271907597));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05189238749151336, 0.04662848449772161));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001626160367043256, 0.0015173354604608978));

                c.maxAchievableForwardVelocity.set(65.56301515333936);
                c.maxAchievableStrafeVelocity.set(51.36550393607606);
                c.naturalForwardDeceleration.set(77.41965912943441);
                c.naturalStrafeDeceleration.set(108.52848009796827);
            }
    );
}