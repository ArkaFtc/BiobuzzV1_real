package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "ConfigProbe")
public class ConfigProbe extends LinearOpMode {
    public static double testValue = 0;
    public static int testInt = 0;
    public static boolean testBool = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        int loops = 0;
        while (opModeIsActive()) {
            loops++;
            telemetry.addData("loops", loops);
            telemetry.addData("runtime", getRuntime());
            telemetry.addData("testValue", testValue);
            telemetry.addData("testInt", testInt);
            telemetry.addData("testBool", testBool);
            telemetry.update();
        }
    }
}