package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoTestOnly")
public class IntakeTestOnly extends LinearOpMode {

    public static volatile double dropDownPos = 0.5;
    public static volatile double gatePos = 0.5;
    public static volatile double flowerPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo dropDown = hardwareMap.get(Servo.class, "dropDown");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        Servo flower = hardwareMap.get(Servo.class, "flowerServo");

        waitForStart();

        while (opModeIsActive()) {
            // Each press = one step (rising edge)
            if (gamepad1.dpadUpWasPressed())       dropDownPos = Math.min(1.0, dropDownPos + 0.02);
            if (gamepad1.dpadDownWasPressed())     dropDownPos = Math.max(0.0, dropDownPos - 0.02);
            if (gamepad1.aWasPressed())            gatePos = Math.min(1.0, gatePos + 0.01);
            if (gamepad1.yWasPressed())            gatePos = Math.max(0.0, gatePos - 0.01);
            if (gamepad1.leftBumperWasPressed())   flowerPos = Math.min(1.0, flowerPos + 0.02);
            if (gamepad1.rightBumperWasPressed())  flowerPos = Math.max(0.0, flowerPos - 0.02);

            dropDown.setPosition(dropDownPos);
            flower.setPosition(flowerPos);
            gate.setPosition(gatePos);

            telemetry.addData("dropDownPos", dropDownPos);
            telemetry.addData("gatePos", gatePos);
            telemetry.addData("flowerPos", flowerPos);
            telemetry.update();
        }
    }
}