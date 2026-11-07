package org.firstinspires.ftc.teamcode.teles;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

public class Tele extends OpMode {
    Robot robot;
    Alliance alliance;
    StateMachine stateMachine;
    public static double targetVelo=1600;
    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double hoodStartPos = 0.5;
    public static double hoodIncrements = 0.02;
    private double hoodPos;
    // Each current read is an extra hub call, so this can be turned off to speed up the loop
    public static boolean showCurrent = false;
    private List<LynxModule> hubs;
    private PIDFController headingPID, secondaryHeadingPID;
    public Pose relocalizePos;
    // How long the flower arm stays down after the trigger is released
    public static double flowerReleaseTime = 0.2;
    public enum States {
        Intake,
        FlowerRelease, Got4,
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
        hoodPos = hoodStartPos;
        hubs = hardwareMap.getAll(LynxModule.class);


        stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(()->{
                    robot.intake.setDropDown(true);
                    robot.intake.setPower(1);
                    robot.shooter.setGate(false);
                })
                .loop(()->{
                    boolean flowerDown = gamepad1.left_trigger > 0.8;
                    // Only lower the arm here; raising it is done by FlowerRelease after a delay
                    if (flowerDown)
                        robot.flowermech.setFlowerArm(true);
                    if (gamepad1.options){
                        robot.intake.setPower(-0.6);
                        robot.intake.setDropDown(false);
                    }
                    else{
                        robot.intake.setIntakePower(1);
                        // Stop the transfer once the back 2 slots are full
                        robot.intake.setTransferPower(robot.intake.hasBack2() ? 0 : 1);
                        if (flowerDown)
                            robot.intake.setDropDownMid();
                        else
                            robot.intake.setDropDown(true);
                    }
                })
                .transition(() -> gamepad1.a, States.HoldBalls)
                .transition(() -> robot.intake.all4(), States.Got4)
                .transition(() -> gamepad1.b, States.OpenGate)
                // Trigger released while the arm is down: hold it a little longer
                .transition(() -> robot.flowermech.isDown && gamepad1.left_trigger <= 0.8, States.FlowerRelease)
                .state(States.FlowerRelease)
                .onEnter(()->{
                    robot.flowermech.setFlowerArm(true);
                    robot.intake.setDropDown(true);
                })
                .loop(()->{
                    robot.intake.setIntakePower(1);
                    robot.intake.setTransferPower(robot.intake.hasBack2() ? 0 : 1);
                })
                .onExit(()-> robot.flowermech.setFlowerArm(false))
                .transition(() -> gamepad1.b, States.OpenGate)
                .transition(() -> gamepad1.a, States.HoldBalls)
                .transition(() -> robot.intake.all4(), States.HoldBalls)
                .transition(() -> gamepad1.left_trigger > 0.8, States.Intake)
                .transitionTimed(flowerReleaseTime, States.Intake)
                .state(States.Got4)
                .onEnter(()->{
                    robot.flowermech.setFlowerArm(false);
                    robot.intake.setDropDown(false);
                    robot.intake.setIntakePower(1);
                    robot.intake.setTransferPower(0);
                    robot.shooter.setGate(false);
                })
                .transition(() -> gamepad1.y, States.Intake)
                .transition(() -> gamepad1.b, States.OpenGate)
                .transitionTimed(0.4,States.HoldBalls)
                .state(States.HoldBalls)
                .onEnter(()->{
                    robot.flowermech.setFlowerArm(false);
                    robot.intake.setDropDown(false);
                    robot.intake.setIntakePower(0.2);
                    robot.intake.setTransferPower(0);
                    robot.shooter.setGate(false);
                })
                .transition(() -> gamepad1.y, States.Intake)
                .transition(() -> gamepad1.b, States.OpenGate)

                .state(States.OpenGate)
                .onEnter(()->{
                    robot.flowermech.setFlowerArm(false);
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
                .transitionTimed(0.3, States.Intake)

                .build();
    }
    public void init_loop() {
        robot.follower.setPose(Robot.savedPose);
        robot.update();
        telemetry.addData("Shooter Target", target);
        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.update();
    }
    private static double round2(double value) {
        return Math.round(value * 100) / 100.0;
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
        if (gamepad2.dpadUpWasPressed())
            hoodPos += hoodIncrements;
        if (gamepad2.dpadDownWasPressed())
            hoodPos -= hoodIncrements;
        // Keep within the servo limits so extra presses past the end don't build up
        hoodPos = Range.clip(hoodPos, Shooter.hoodLowerBound, Shooter.hoodUpperBound);
        robot.shooter.setHood(hoodPos);

        stateMachine.update();

        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.addData("Shooter Target", robot.shooter.getTargetVelo());
        telemetry.addData("Shooter Velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("Hood Pos", hoodPos);
        telemetry.addData("Has Back 2 (filtered)", robot.intake.hasBack2());
        telemetry.addData("Back 2 Raw", robot.intake.ball2());
        telemetry.addData("Back 2 Filter Value", round2(robot.intake.getBack2Value()));
        telemetry.addData("Has Ball 3 (filtered)", robot.intake.hasBall3());
        telemetry.addData("Ball 3 Filter Value", round2(robot.intake.getBall3Value()));
        telemetry.addData("Has Ball 4 (filtered)", robot.intake.hasBall4());
        telemetry.addData("Ball 4 Filter Value", round2(robot.intake.getBall4Value()));
        if (showCurrent) {
            double frontCurrent = robot.intake.getFrontCurrent();
            double transferCurrent = robot.intake.getTransferCurrent();
            double shooter1Current = robot.shooter.getShooter1Current();
            double shooter2Current = robot.shooter.getShooter2Current();
            // Whole-robot draw from each hub (includes drive motors and servos)
            double hubTotal = 0;
            for (LynxModule hub : hubs) {
                hubTotal += hub.getCurrent(CurrentUnit.AMPS);
            }
            // Sent as numbers (not formatted text) so Panels can graph them
            telemetry.addData("Front Intake Current (A)", round2(frontCurrent));
            telemetry.addData("Transfer Current (A)", round2(transferCurrent));
            telemetry.addData("Shooter 1 Current (A)", round2(shooter1Current));
            telemetry.addData("Shooter 2 Current (A)", round2(shooter2Current));
            telemetry.addData("Intake + Shooter Total (A)",
                    round2(frontCurrent + transferCurrent + shooter1Current + shooter2Current));
            telemetry.addData("Robot Total Current (A)", round2(hubTotal));
        }
        telemetry.addData("Statemachine State", stateMachine.getState());
        telemetry.addData("Loop time hz", robot.getLoopTimeHz());

        telemetry.update();
    }
}