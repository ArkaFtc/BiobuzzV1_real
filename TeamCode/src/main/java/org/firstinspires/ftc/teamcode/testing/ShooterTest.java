package org.firstinspires.ftc.teamcode.testing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Configurable
@TeleOp
public class ShooterTest extends LinearOpMode {
    Shooter shooter;

    public static double hoodPos = 0.5;
    public static double shooterPower = 0;
    public static boolean upperGateOpen = false;
    public static double turretBigStep = 0.05;
    public static double turretSmallStep = 0.005;
    public static double diffTurretStep = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);

        // Starts centered every run
        double turretPos = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed())    turretPos += turretBigStep;
            if (gamepad1.dpadDownWasPressed())  turretPos -= turretBigStep;
            if (gamepad1.dpadRightWasPressed()) turretPos += turretSmallStep;
            if (gamepad1.dpadLeftWasPressed())  turretPos -= turretSmallStep;
            turretPos = Range.clip(turretPos, Shooter.turretLowerBound, Shooter.turretUpperBound);
            if (gamepad1.yWasPressed()) Shooter.diffTurret += diffTurretStep;
            if (gamepad1.aWasPressed()) Shooter.diffTurret -= diffTurretStep;
            shooter.setTurretPos(turretPos);

            shooter.setGate(upperGateOpen);
            shooter.setHood(hoodPos);
            shooter.updateDirect(shooterPower);

            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Turret Pos", turretPos);
            telemetry.addData("Diff Turret", Shooter.diffTurret);
            telemetry.update();
        }
    }
}