package org.firstinspires.ftc.teamcode.teles;


import com.bylazar.configurables.annotations.Configurable;
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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;
import java.util.List;

@Configurable
public class Tele extends OpMode {
    Robot robot;
    Alliance alliance;
    StateMachine stateMachine;
    public static double targetVelo=1600;
    public static double powerOffsetIncrements = 20;
    public static double hoodStartPos = 0.5;
    public static double hoodIncrements = 0.02;
    private double hoodPos;
    public static boolean showCurrent = true;
    private List<LynxModule> hubs;
    public static double headingKP = 1.60;
    public static double headingKD = 0.079;
    private boolean headingLocked = false;
    private double prevHeadingError = 0;
    private long prevHeadingTime = 0;
    public Pose relocalizePos;
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
            relocalizePos = new Pose(133, 105, Math.toRadians(180));
        } else {
            relocalizePos = new Pose(8, 37, Math.toRadians(0));
        }
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot = new Robot(hardwareMap, alliance);
        robot.update();
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
        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.update();
    }
    private static double round2(double value) {
        return Math.round(value * 100) / 100.0;
    }
    // Wraps an angle in radians to the range -pi..pi
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // PD turn power that points the robot at the closest goal. The target is recomputed
    // every loop, so it keeps tracking the goal while driving.
    private double headingLockTurn() {
        Pose pose = robot.follower.pose();
        Pose goal = robot.shooter.getClosestGoal(pose, alliance);
        double targetHeading = Math.atan2(goal.y() - pose.y(), goal.x() - pose.x());
        double error = wrapAngle(targetHeading - pose.heading());

        long now = System.nanoTime();
        double derivative = 0;
        // Skip the D term on the first loop after pressing so it doesn't kick
        if (prevHeadingTime != 0) {
            double dt = (now - prevHeadingTime) / 1e9;
            if (dt > 0)
                derivative = wrapAngle(error - prevHeadingError) / dt;
        }
        prevHeadingError = error;
        prevHeadingTime = now;

        return Range.clip(headingKP * error + headingKD * derivative, -1, 1);
    }

    public void start() {
        stateMachine.start();
    }
    public void loop() {
        robot.update();
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            forward *= 0.4;
            strafe *= 0.4;
            turn *= 0.4;
        }
        double turnPower = -turn;
        headingLocked = gamepad1.right_trigger > 0.5;
        if (headingLocked)
            turnPower = headingLockTurn();
        else
            prevHeadingTime = 0;
        robot.follower.manual(-forward, -strafe, turnPower);
        if (gamepad1.leftBumperWasPressed()) {
            robot.follower.setPose(relocalizePos);
            Shooter.powerOffset = 0;
        }

        // Dpads nudge the lookup-table values up or down
        if (gamepad1.dpadDownWasPressed())
            Shooter.powerOffset -= powerOffsetIncrements;
        if (gamepad1.dpadUpWasPressed())
            Shooter.powerOffset += powerOffsetIncrements;
        if (gamepad2.dpadUpWasPressed())
            Shooter.hoodOffset += hoodIncrements;
        if (gamepad2.dpadDownWasPressed())
            Shooter.hoodOffset -= hoodIncrements;
        // Sets shooter velocity and hood from the distance to the goal (plus the offsets above)
        robot.shooter.aimAtTarget(robot.follower.pose(),alliance);
        stateMachine.update();

        telemetry.addData("Current Pos", robot.follower.pose());
        Pose pose = robot.follower.pose();
        telemetry.addData("Left Hive Distance", round2(robot.shooter.getLeftGoalDistance(pose, alliance)));
        telemetry.addData("Right Hive Distance", round2(robot.shooter.getRightGoalDistance(pose, alliance)));
        telemetry.addData("Aiming At", robot.shooter.getAimedGoalSide(pose, alliance));
        telemetry.addData("Heading Locked", headingLocked);
        telemetry.addData("Heading Error (deg)", headingLocked ? round2(Math.toDegrees(prevHeadingError)) : 0);
        telemetry.addData("Angle and distance", Arrays.toString(robot.shooter.getAngleDistance(robot.follower.pose(), alliance)));
        telemetry.addData("Shooter Target", robot.shooter.getTargetVelo());
        telemetry.addData("Shooter Velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("Power Offset", Shooter.powerOffset);
        telemetry.addData("Hood Offset", Shooter.hoodOffset);
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