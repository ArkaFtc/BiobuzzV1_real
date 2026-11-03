package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp(name = "ShooterPIDTesting")
public class  ShooterPidTesting extends LinearOpMode {

    // Step sizes per button press
    public static double kPStep = 0.0005;
    public static double kSStep = 0.005;
    public static double kVStep = 0.00001;
    public static double veloStep = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        Shooter shooter = new Shooter(hardwareMap);

        // Start from the Shooter class values. These are local copies, so tuning here
        // doesn't change what teleop uses - copy the final numbers into Shooter.java.
        double kP = Shooter.kP;
        double kS = Shooter.kS;
        double kV = Shooter.kV;
        double targetVelo = 0;

        boolean prevLeftTrigger = false;
        boolean prevRightTrigger = false;

        waitForStart();

        while (opModeIsActive()) {
            // Each press = one step (rising edge)
            if (gamepad1.dpadUpWasPressed())       kP += kPStep;
            if (gamepad1.dpadDownWasPressed())     kP -= kPStep;
            if (gamepad1.yWasPressed())            kS += kSStep;
            if (gamepad1.aWasPressed())            kS -= kSStep;
            if (gamepad1.rightBumperWasPressed())  kV += kVStep;
            if (gamepad1.leftBumperWasPressed())   kV -= kVStep;

            // Triggers are analog, so treat pulling past halfway as one press
            boolean leftTrigger = gamepad1.left_trigger > 0.5;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;
            if (rightTrigger && !prevRightTrigger) targetVelo += veloStep;
            if (leftTrigger && !prevLeftTrigger)   targetVelo -= veloStep;
            prevLeftTrigger = leftTrigger;
            prevRightTrigger = rightTrigger;

            kP = Math.max(0, kP);
            kS = Math.max(0, kS);
            kV = Math.max(0, kV);
            targetVelo = Math.max(0, targetVelo);

            shooter.setGains(kP, kS, kV);
            shooter.setTargetVelocity(targetVelo);
            shooter.update();

            double actualVelo = shooter.getCurrentVelocity();

            // Sent as numbers so Panels can graph them
            telemetry.addData("Target Velocity", targetVelo);
            telemetry.addData("Actual Velocity", actualVelo);
            telemetry.addData("Error", targetVelo - actualVelo);
            telemetry.addData("kP", kP);
            telemetry.addData("kS", kS);
            telemetry.addData("kV", kV);
            telemetry.addLine("dpad up/down = kP, Y/A = kS, RB/LB = kV, RT/LT = target velo");
            telemetry.update();
        }
    }
}
