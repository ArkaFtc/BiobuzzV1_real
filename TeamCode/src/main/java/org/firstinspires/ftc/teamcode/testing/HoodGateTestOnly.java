package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp(name = "HoodGateTestOnly")
public class HoodGateTestOnly extends LinearOpMode {

    public static double hoodStep = 0.02;
    public static double gateStep = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood = hardwareMap.get(Servo.class, "hood");
        Servo gate = hardwareMap.get(Servo.class, "gate");

        // Both start at 0.5 every time the program runs
        double hoodPos = 0.5;
        double gatePos = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            // Each press = one step (rising edge)
            if (gamepad1.yWasPressed())        hoodPos += hoodStep;
            if (gamepad1.aWasPressed())        hoodPos -= hoodStep;
            if (gamepad1.dpadUpWasPressed())   gatePos += gateStep;
            if (gamepad1.dpadDownWasPressed()) gatePos -= gateStep;

            // Hood stays inside the Shooter's safe range; gate can use the full servo range
            hoodPos = Range.clip(hoodPos, Shooter.hoodLowerBound, Shooter.hoodUpperBound);
            gatePos = Range.clip(gatePos, 0.0, 1.0);

            hood.setPosition(hoodPos);
            gate.setPosition(gatePos);

            telemetry.addData("hoodPos", "%.2f", hoodPos);
            telemetry.addData("gatePos", "%.2f", gatePos);
            telemetry.addLine("Y/A = hood up/down, dpad up/down = gate");
            telemetry.update();
        }
    }
}
