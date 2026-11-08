package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Alliance;

// Same intake/shoot state machine as TeleOp, locked to red, with manual shooter velocity and hood.
// Controls (gamepad1): A = stop/hold, Y = restart intake, B = shoot,
// dpad up/down = target velocity, dpad left/right = hood, left bumper = relocalize.
@Configurable
@TeleOp
public class RedStateMachineTesting extends OpMode {
    Robot robot;
    StateMachine stateMachine;

    public static double targetVelo = 1600;
    public static double veloIncrements = 20;
    public static double hoodPos = 0.5;
    public static double hoodIncrements = 0.02;

    private final Pose relocalizePos = new Pose(8, 37, Math.toRadians(0));

    public enum States {
        Intake,
        Got4,
        HoldBalls,
        OpenGate,
        Shoot
    }

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot = new Robot(hardwareMap, Alliance.RED);
        robot.update();
        robot.follower.setPose(Robot.savedPose);

        stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(() -> {
                    robot.flowermech.setFlowerArm(false);
                    robot.intake.setDropDown(true);
                    robot.intake.setPower(1);
                    robot.shooter.setGate(false);
                })
                .loop(() -> {
                    robot.intake.setIntakePower(1);
                    // Stop the transfer once the back 2 slots are full
                    robot.intake.setTransferPower(robot.intake.hasBack2() ? 0 : 1);
                })
                .transition(() -> gamepad1.a, States.HoldBalls)
                .transition(() -> robot.intake.all4(), States.Got4)
                .transition(() -> gamepad1.b, States.OpenGate)

                .state(States.Got4)
                .onEnter(() -> {
                    robot.intake.setDropDown(false);
                    robot.intake.setIntakePower(1);
                    robot.intake.setTransferPower(0);
                    robot.shooter.setGate(false);
                })
                .transition(() -> gamepad1.y, States.Intake)
                .transition(() -> gamepad1.b, States.OpenGate)
                .transitionTimed(0.4, States.HoldBalls)

                .state(States.HoldBalls)
                .onEnter(() -> {
                    robot.intake.setDropDown(false);
                    robot.intake.setIntakePower(0.2);
                    robot.intake.setTransferPower(0);
                    robot.shooter.setGate(false);
                })
                .transition(() -> gamepad1.y, States.Intake)
                .transition(() -> gamepad1.b, States.OpenGate)

                .state(States.OpenGate)
                .onEnter(() -> {
                    robot.intake.setDropDown(true);
                    robot.intake.setPower(0);
                    robot.shooter.setGate(true);
                })
                .transitionTimed(0.1, States.Shoot)

                .state(States.Shoot)
                .onEnter(() -> {
                    robot.intake.setPower(1);
                    robot.shooter.setGate(true);
                })
                .transitionTimed(0.3, States.Intake)

                .build();
    }

    @Override
    public void init_loop() {
        robot.follower.setPose(Robot.savedPose);
        robot.update();
        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.update();
    }

    private static double round2(double value) {
        return Math.round(value * 100) / 100.0;
    }

    @Override
    public void start() {
        stateMachine.start();
    }

    @Override
    public void loop() {
        robot.update();

        if (gamepad1.leftBumperWasPressed())
            robot.follower.setPose(relocalizePos);

        if (gamepad1.dpadUpWasPressed())
            targetVelo += veloIncrements;
        if (gamepad1.dpadDownWasPressed())
            targetVelo -= veloIncrements;
        if (gamepad1.dpadRightWasPressed())
            hoodPos += hoodIncrements;
        if (gamepad1.dpadLeftWasPressed())
            hoodPos -= hoodIncrements;
        targetVelo = Math.max(targetVelo, 0);
        // Keep within the servo limits so extra presses past the end don't build up
        hoodPos = Range.clip(hoodPos, Shooter.hoodLowerBound, Shooter.hoodUpperBound);

        robot.shooter.setTargetVelocity(targetVelo);
        robot.shooter.setHood(hoodPos);

        stateMachine.update();

        telemetry.addData("Target Velocity", targetVelo);
        telemetry.addData("Shooter Velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("Hood Pos", hoodPos);
        Pose pose = robot.follower.pose();
        telemetry.addData("Current Pos", pose);
        telemetry.addData("Left Hive Distance", round2(robot.shooter.getLeftGoalDistance(pose, Alliance.RED)));
        telemetry.addData("Right Hive Distance", round2(robot.shooter.getRightGoalDistance(pose, Alliance.RED)));
        telemetry.addData("Aiming At", robot.shooter.getAimedGoalSide(pose, Alliance.RED));
        telemetry.addData("Statemachine State", stateMachine.getState());
        telemetry.update();
    }
}
