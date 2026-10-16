package org.firstinspires.ftc.teamcode.teles;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

import java.util.Arrays;

public class Tele extends OpMode {
    Robot robot;
    Alliance alliance;
    StateMachine stateMachine;
    public static double targetVelo=1600;
    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double hoodOffsetIncrements = 0.02;
    private PIDFController headingPID, secondaryHeadingPID;
    public Pose relocalizePos;
    public enum States {
        Intake,
        HoldBalls,
        OpenGate,
        Shoot
    }
    public Tele(Alliance alliance) {
        this.alliance = alliance;
    }
    public void init() {
        if (alliance.equals(Alliance.BLUE)) {
            target = Shooter.Goal.BLUE;
            relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));
        } else {
            target = Shooter.Goal.RED;
            relocalizePos = new Pose(-14.5, 56, Math.toRadians(90));
        }
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot = new Robot(hardwareMap, alliance);
        robot.update();
        headingPID = new PIDFController(1.4426, 0, 0.2179, 0);
        secondaryHeadingPID = new PIDFController(0.6412, 0 ,0.1141, 0);

        robot.follower.setPose(Robot.savedPose);


        stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(()->{
                    robot.intake.setDropDown(true);
                    robot.intake.setPower(1);
                    robot.shooter.setGate(false);
                })
                .loop(()->{
                    robot.flowermech.setFlowerArm(gamepad1.left_trigger > 0.8);
                    if (gamepad1.options){
                        robot.intake.setPower(-0.4);
                    }
                    else{
                        robot.intake.setPower(1);
                    }
                })
                .transition(() -> gamepad1.a, States.HoldBalls)
                .transition(() -> robot.intake.all4(), States.HoldBalls)
                .transition(() -> gamepad1.b, States.OpenGate)

                .state(States.HoldBalls)
                .onEnter(()->{
                    robot.intake.setDropDown(false);
                    robot.intake.setPower(0);
                    robot.shooter.setGate(false);
                })
                .transition(() -> gamepad1.y, States.Intake)
                .transition(() -> gamepad1.b, States.OpenGate)

                .state(States.OpenGate)
                .onEnter(()->{
                    robot.intake.setDropDown(true);
                    robot.intake.setPower(0);
                    robot.shooter.setGate(true);
                })
                .transitionTimed(0.1, States.Shoot)
                .state(States.Shoot)
                .onEnter(()->{
                    robot.intake.setPower(1);
                    robot.shooter.setGate(true);
                })
                .transitionTimed(0.4, States.Intake)

                .build();
    }
    public void init_loop() {
        robot.follower.setPose(Robot.savedPose);
        robot.update();
        telemetry.addData("Shooter Target", target);
        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.update();
    }
    public void start() {
        stateMachine.start();
    }
    public void loop() {
        robot.update();
        telemetry.addData("Angle and distance:", Arrays.toString(robot.shooter.getAngleDistance(robot.follower.pose(), target)));
        robot.shooter.setTargetVelocity(targetVelo);

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            forward *= 0.4;
            strafe *= 0.4;
            turn *= 0.4;
        }
        robot.follower.manual(-forward, -strafe, -turn);
        if (gamepad1.leftBumperWasPressed()) {
            robot.follower.setPose(relocalizePos);
            Shooter.powerOffset = 0;
        }

        if (gamepad1.dpadDownWasPressed())
            targetVelo -= powerOffsetIncrements;
        if (gamepad1.dpadUpWasPressed())
            targetVelo += powerOffsetIncrements;
        if (gamepad2.yWasPressed())
            Shooter.hoodOffset -= hoodOffsetIncrements;
        if (gamepad2.aWasPressed())
            Shooter.hoodOffset += hoodOffsetIncrements;

        stateMachine.update();

        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.addData("Shooter Target", robot.shooter.getTargetVelo());
        telemetry.addData("Shooter Velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("Statemachine State", stateMachine.getState());
        telemetry.addData("Loop time hz", robot.getLoopTimeHz());

        telemetry.update();
    }
}
