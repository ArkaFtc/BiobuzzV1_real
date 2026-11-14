package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.teles.Tele;
import org.firstinspires.ftc.teamcode.utils.Alliance;

@Configurable
@TeleOp(name = "HeadingPIDTesting")
public class HeadingPidTesting extends OpMode {

    // Step sizes per dpad press
    public static double kPStep = 0.05;
    public static double kDStep = 0.01;
    public static double targetHeadingDeg = 0;

    private Robot robot;
    // Local copies so tuning here doesn't change teleop - copy the final numbers into Tele.java
    private double kP = Tele.headingKP;
    private double kD = Tele.headingKD;
    private double prevError = 0;
    private long prevTime = 0;

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot = new Robot(hardwareMap, Alliance.RED);
        robot.update();
    }

    @Override
    public void start() {
        // Whatever way the robot faces when you press play becomes 0 deg
        robot.follower.setPose(new Pose(0, 0, 0));
        prevTime = 0;
    }

    // Wraps an angle in radians to the range -pi..pi
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // Same PD as teleop, but aiming at a fixed heading instead of the goal
    private double headingLockTurn(double error) {
        long now = System.nanoTime();
        double derivative = 0;
        // Skip the D term on the first loop after pressing so it doesn't kick
        if (prevTime != 0) {
            double dt = (now - prevTime) / 1e9;
            if (dt > 0)
                derivative = wrapAngle(error - prevError) / dt;
        }
        prevError = error;
        prevTime = now;
        return Range.clip(kP * error + kD * derivative, -1, 1);
    }

    @Override
    public void loop() {
        robot.update();

        if (gamepad1.dpadUpWasPressed())    kP += kPStep;
        if (gamepad1.dpadDownWasPressed())  kP -= kPStep;
        if (gamepad1.dpadRightWasPressed()) kD += kDStep;
        if (gamepad1.dpadLeftWasPressed())  kD -= kDStep;
        kP = Math.max(0, kP);
        kD = Math.max(0, kD);

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turnPower = -gamepad1.right_stick_x;

        double heading = robot.follower.pose().heading();
        double error = wrapAngle(Math.toRadians(targetHeadingDeg) - heading);

        boolean locked = gamepad1.right_trigger > 0.5;
        if (locked)
            turnPower = headingLockTurn(error);
        else
            prevTime = 0;

        robot.follower.manual(-forward, -strafe, turnPower);

        // Sent as numbers so Panels can graph them
        telemetry.addData("Heading Locked", locked);
        telemetry.addData("Target Heading (deg)", targetHeadingDeg);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Error (deg)", Math.toDegrees(error));
        telemetry.addData("Turn Power", turnPower);
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addLine("RT = lock to target, dpad up/down = kP, dpad right/left = kD");
        telemetry.update();
    }
}
