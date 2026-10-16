package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlowerMech;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp
public class FullTest extends LinearOpMode {
    Intake intake;
    FlowerMech flowermech;
    Shooter shooter;

    public static double intakePower = 0.5;
    public static double transferPower = 0.0;
    public static double hoodPos = 0.5;
    public static int shooterTargetVelocity = 0;
    public static boolean flowerdown = false;
    public static boolean dropDown = false;
    public static boolean upperGateOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intake = new Intake(hardwareMap);
        flowermech = new FlowerMech(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intake.setIntakePower(intakePower);
            intake.setTransferPower(transferPower);
            intake.setDropDown(dropDown);
            shooter.setTargetVelocity(shooterTargetVelocity);
            flowermech.setFlowerArm(flowerdown);
            shooter.setGate(upperGateOpen);
            shooter.setHood(hoodPos);


            flowermech.update();
            shooter.update();

            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Shooter Target Velocity", shooterTargetVelocity);
            telemetry.addData("Intake All 4?", intake.all4());
            telemetry.addData("ball 1", intake.ball1());
            telemetry.addData("ball 2", intake.ball2());
            telemetry.addData("ball 3", intake.ball3());
            telemetry.addData("ball 4", intake.ball4());
            telemetry.update();
        }
    }
}