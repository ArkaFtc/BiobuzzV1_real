package org.firstinspires.ftc.teamcode.autos.baronway;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;

// Red only: the RedTipsNoTurretPaths points are red-side
@Configurable
@Autonomous(name = "redAutoNoTurret")
public class redAutoNoTurret extends CommandOpMode {
    public static double pathTimeoutMs = 4000;
    public static double spinUpTimeoutMs = 1500;
    public static double velocityTolerance = 40;
    public static double gateOpenMs = 100;
    public static double shootMs = 400;
    public static double flowerDownCompletion = 0.5;
    public static double idleFrontPower = 0.6;

    public static double leftGoalX = 58, leftGoalY = 86.5;
    public static double rightGoalX = 58, rightGoalY = 53;

    final Alliance alliance = Alliance.RED;
    RedTipsNoTurretPaths paths;
    Robot robot;
    // When true, the loop runs the transfer until the back 2 slots are full (same as tele)
    boolean intaking = false;

    @Override
    public void init() {
        reset();
        robot = new Robot(hardwareMap, alliance);
        paths = new RedTipsNoTurretPaths();

        robot.pose(paths.start());
        robot.shooter.setGate(false);
        robot.intake.setDropDown(false);
        robot.intake.setPower(0);
        robot.flowermech.setFlowerArm(false);

        schedule(
                sequential(
                        instant(() -> {
                            robot.pose(paths.start());
                            robot.intake.setIntakePower(idleFrontPower);
                        }),

                        // Preloads
                        drive(paths.push()),
                        drive(paths.pushToShoot()),
                        drive(paths.turnToShoot1()),
                        waitMs(100),
                        shoot(),

                        // Garden
                        intakeOn(),
                        drive(paths.intakeGarden()),
                        hold(),
                        drive(paths.gardenToShoot2()),
                        shoot(),

                        // Flower 1
                        intakeOn(),
                        drive(paths.toFlower1()),
                        flowerDrive(paths.flower1()),
                        hold(),
                        drive(paths.flower1ToShoot()),
                        shoot(),

                        // Flower 2
                        intakeOn(),
                        drive(paths.toFlower2()),
                        flowerDrive(paths.flower2()),
                        hold(),
                        drive(paths.flower2ToShoot()),
                        shoot(),

                        // Park
                        stopAll(),
                        drive(paths.park())
                ),
                infinite(() -> {
                    robot.update();
                    // Sets flywheel velocity and hood from distance to the auto goal
                    Pose goal = closestGoal(robot.pose());
                    robot.shooter.aimAtTarget(robot.pose(), goal);
                    telemetry.addData("aiming at", goal);
                    if (intaking)
                        robot.intake.setTransferPower(robot.intake.hasBack2() ? 0 : 1);

                    telemetry.addData("pose", robot.pose());
                    telemetry.addData("shooter target", robot.shooter.getTargetVelo());
                    telemetry.addData("shooter velocity", robot.shooter.getCurrentVelocity());
                    telemetry.update();
                })
        );
    }

    @Override
    public void init_loop() {
        robot.update();
        telemetry.addData("pose", robot.pose());
        telemetry.update();
    }

    Pose closestGoal(Pose pose) {
        Pose left = new Pose(leftGoalX, leftGoalY);
        Pose right = new Pose(rightGoalX, rightGoalY);
        double leftDist = Math.hypot(left.x() - pose.x(), left.y() - pose.y());
        double rightDist = Math.hypot(right.x() - pose.x(), right.y() - pose.y());
        return leftDist <= rightDist ? left : right;
    }

    CommandBuilder drive(Path path) {
        return follow(robot.follower, path).raceWith(waitMs(pathTimeoutMs));
    }

    // Drives a flower path, dropping the arm partway through
    CommandBuilder flowerDrive(Path path) {
        return follow(robot.follower, path)
                .with(
                        waitUntil(() -> robot.follower.completion() > flowerDownCompletion)
                                .then(instant(() -> {
                                    robot.flowermech.setFlowerArm(true);
                                    robot.intake.setDropDownMid();
                                }))
                )
                .raceWith(waitMs(pathTimeoutMs));
    }

    CommandBuilder intakeOn() {
        return instant(() -> {
            robot.flowermech.setFlowerArm(false);
            robot.shooter.setGate(false);
            robot.intake.setDropDown(true);
            robot.intake.setIntakePower(1);
            intaking = true;
        });
    }

    CommandBuilder hold() {
        return instant(() -> {
            intaking = false;
            robot.flowermech.setFlowerArm(false);
            robot.shooter.setGate(false);
            robot.intake.setDropDown(false);
            robot.intake.setIntakePower(idleFrontPower);
            robot.intake.setTransferPower(0);
        });
    }

    // Same order as tele: open gate, then feed everything through
    CommandBuilder shoot() {
        return sequential(
                waitUntil(() -> Math.abs(robot.shooter.getTargetVelo() - robot.shooter.getCurrentVelocity()) < velocityTolerance)
                        .raceWith(waitMs(spinUpTimeoutMs)),
                instant(() -> {
                    intaking = false;
                    robot.flowermech.setFlowerArm(false);
                    robot.intake.setDropDown(true);
                    robot.intake.setIntakePower(idleFrontPower);
                    robot.intake.setTransferPower(0);
                    robot.shooter.setGate(true);
                }),
                waitMs(gateOpenMs),
                instant(() -> robot.intake.setPower(1)),
                waitMs(shootMs),
                instant(() -> {
                    robot.intake.setIntakePower(idleFrontPower);
                    robot.intake.setTransferPower(0);
                    robot.shooter.setGate(false);
                })
        );
    }

    CommandBuilder stopAll() {
        return instant(() -> {
            intaking = false;
            robot.flowermech.setFlowerArm(false);
            robot.shooter.setGate(false);
            robot.intake.setDropDown(false);
            robot.intake.setIntakePower(idleFrontPower);
            robot.intake.setTransferPower(0);
        });
    }

    @Override
    public void stop() {
        robot.saveEnd();
        super.stop();
    }
}
