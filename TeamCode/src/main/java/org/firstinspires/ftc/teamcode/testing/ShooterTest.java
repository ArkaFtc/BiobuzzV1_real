package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Configurable
@TeleOp
public class ShooterTest extends LinearOpMode {
    Shooter shooter;

    public static double hoodPos = 0.5;
    public static int shooterPower = 0;
    public static boolean upperGateOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            shooter.setDirectPower(shooterPower);
            shooter.setGate(upperGateOpen);
            shooter.setHood(hoodPos);

            shooter.update();
            telemetry.update();
        }
    }
}
