package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMech;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp(name = "IntakeTestOnly")
public class IntakeTestOnly extends LinearOpMode {

    // Defaults: drop down lowered, gate closed, flower arm idle (up)
    public static volatile double dropDownPos = Intake.dropDownPos;
    public static volatile double gatePos = Shooter.upperGateClosedPos;
    public static volatile double flowerPos = FlowerMech.flowerIdlePos;

    @Override
    public void runOpMode() throws InterruptedException {
        // Send telemetry to both the driver station and Panels
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        Servo dropDown = hardwareMap.get(Servo.class, "dropDown");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        Servo flower = hardwareMap.get(Servo.class, "flowerServo");
        DcMotorEx frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        DcMotorEx transferIntake = hardwareMap.get(DcMotorEx.class, "transferIntake");
        // Read fresh sensor values every loop (teleop uses MANUAL caching, which would freeze reads here)
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Only used to read the beam breaks; the motors and servos above are driven directly
        Intake intake = new Intake(hardwareMap);

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

            // Triggers give 0..1; negated to match Intake subsystem's "intake in" direction
            double frontPower = gamepad1.left_trigger;
            double transferPower = gamepad1.right_trigger;
            frontIntake.setPower(-frontPower);
            transferIntake.setPower(-transferPower);

            telemetry.addData("frontIntakePower", frontPower);
            telemetry.addData("transferIntakePower", transferPower);
            double frontCurrent = frontIntake.getCurrent(CurrentUnit.AMPS);
            double transferCurrent = transferIntake.getCurrent(CurrentUnit.AMPS);
            // Sent as numbers (not formatted text) so Panels can graph them
            telemetry.addData("frontIntakeCurrent (A)", Math.round(frontCurrent * 100) / 100.0);
            telemetry.addData("transferIntakeCurrent (A)", Math.round(transferCurrent * 100) / 100.0);
            telemetry.addData("intakeTotalCurrent (A)", Math.round((frontCurrent + transferCurrent) * 100) / 100.0);
            telemetry.addData("ball 1", intake.ball1());
            telemetry.addData("ball 2", intake.ball2());
            telemetry.addData("ball 3", intake.ball3());
            telemetry.addData("ball 4", intake.ball4());
            telemetry.addData("dropDownPos", dropDownPos);
            telemetry.addData("gatePos", gatePos);
            telemetry.addData("flowerPos", flowerPos);
            telemetry.update();
        }
    }
}