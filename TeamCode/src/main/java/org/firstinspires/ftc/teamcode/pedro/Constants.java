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
        c.manualBrakeMode.set(true);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.2090921477070005);
        c.yPodOffset.set(-4.386894346222165);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29533412918685914);
                Controller secondaryTranslationalForward = Controller.proportional(0.10911806926127325);
                Controller primaryTranslationalLateral = Controller.proportional(0.4186748634907785);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15468917486145262);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013904533161362472));
                c.brake.set(Controller.proportionalFeedforward(0.0118188531871581));

                c.headingFeedback.set(Controller.proportional(4.084043007674482));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.049161115269360824, 0.003709614435580893));

                c.linearBrakeCoefficients.set(Matrix.diag(0.050633124313216024, 0.05136019103453072));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002118111687422172, 0.0017029232044241153));

                c.maxAchievableForwardVelocity.set(71.09664887807439);
                c.maxAchievableStrafeVelocity.set(58.25634440779419);
                c.naturalForwardDeceleration.set(45.021680344771994);
                c.naturalStrafeDeceleration.set(82.43200942857636);
            }
    );
}