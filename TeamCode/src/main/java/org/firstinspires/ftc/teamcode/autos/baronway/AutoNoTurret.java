/*package org.firstinspires.ftc.teamcode.autos.baronway;

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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;

// Shared auto; redAutoNoTurret / blueAutoNoTurret pick the alliance (blue paths are mirrored in TipsNoTurretPaths)
@Configurable
public abstract class AutoNoTurret extends CommandOpMode {
    public static double pathTimeoutMs = 4000;
    public static double spinUpTimeoutMs = 1500;
    public static double velocityTolerance = 40;
    public static double gateOpenMs = 100;
    public static double shootMs = 1000;
    public static double shootFeedPower = 0.7;
    public static double flowerDownCompletion = 0.5;
    // How long the flower arm stays down before lifting and moving on
    public static double flowerArmDownMs = 3000;
    // Path completion on the way back from a flower at which the intake dropdown goes down
    public static double flower1ReturnDropCompletion = 0.1;
    public static double flower2ReturnDropCompletion = 0.5;
    public static double turnToleranceDeg = 5;
    public static double turnTimeoutMs = 1500;
    public static double idleFrontPower = 0.6;
    // Speed cap (in/s) for the garden intake path; max achievable forward is ~71
    public static double gardenIntakeSpeed = 30;

    final Alliance alliance;
    TipsNoTurretPaths paths;
    Robot robot;
    boolean intaking = false;

    public AutoNoTurret(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        reset();
        robot = new Robot(hardwareMap, alliance);
        paths = new TipsNoTurretPaths(alliance);

        robot.pose(paths.start());
        robot.shooter.setGate(false);
        robot.intake.setDropDown(false);
        robot.intake.setPower(0);
        robot.flowermech.setFlowerArm(false);
    }

    // Scheduled on start, not init: Scheduler.schedule starts the first command immediately,
    // which turned the intake on during init
    @Override
    public void start() {
        schedule(
                sequential(
                        instant(() -> {
                            robot.pose(paths.start());
                            robot.intake.setIntakePower(idleFrontPower);
                        }),

                        // Preloads
                        drive(paths.push()),
                        drive(paths.pushToShoot()),
                        hold(),
                        shoot(),

                        // Garden
                        intakeOn(),
                        drive(paths.intakeGarden().with(Constants.foresightConfig.maxPathSpeed.at(gardenIntakeSpeed))),
                        drive(paths.gardenToShoot2()),
                        turnTo(paths.shoot2Turned()),
                        shoot(),

                        // Flower 1
                        intakeOn(),
                        drive(paths.toFlower1()),
                        flowerDrive(paths.flower1()),
                        flowerReturn(paths.flower1ToShoot(), flower1ReturnDropCompletion),
                        hold(),
                        shoot(),

                        // Flower 2
                        intakeOn(),
                        drive(paths.toFlower2()),
                        flowerDrive(paths.flower2()),
                        hold(),
                        flowerReturn(paths.flower2ToShoot(), flower2ReturnDropCompletion),
                        shoot(),

                        // Park
                        stopAll(),
                        drive(paths.park())
                ),
                infinite(() -> {
                    robot.update();
                    // Turret and flywheel velocity/hood always track the closest goal, same as tele
                    robot.shooter.aimAtTarget(robot.pose(), alliance);
                    telemetry.addData("turret angle (deg)", robot.shooter.getAngleDistance(robot.pose(), alliance)[0]);
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

    CommandBuilder drive(Path path) {
        return follow(robot.follower, path).raceWith(waitMs(pathTimeoutMs));
    }

    // Drives a flower path, drops the arm partway through, and moves on flowerArmDownMs after the drop
    // instead of waiting for the path to finish (which can stall against the wall until the timeout)
    CommandBuilder flowerDrive(Path path) {
        return sequential(
                instant(() -> robot.follower.follow(path)),
                waitUntil(() -> robot.follower.completion() > flowerDownCompletion)
                        .raceWith(waitMs(pathTimeoutMs)),
                instant(() -> {
                    robot.flowermech.setFlowerArm(true);
                    robot.intake.setDropDownMid();
                }),
                waitMs(flowerArmDownMs)
        );
    }

    // Drives back from a flower, dropping the intake dropdown once the robot has started moving
    CommandBuilder flowerReturn(Path path, double dropCompletion) {
        return follow(robot.follower, path)
                .with(
                        waitUntil(() -> robot.follower.completion() > dropCompletion)
                                .then(instant(() -> robot.intake.setDropDown(true)))
                )
                .raceWith(waitMs(pathTimeoutMs));
    }

    // Turns in place to the target pose's heading (follower picks the shortest direction)
    CommandBuilder turnTo(Pose target) {
        return sequential(
                instant(() -> robot.follower.hold(target)),
                waitUntil(() -> {
                    double error = Math.toDegrees(robot.pose().heading() - target.heading());
                    error = ((error % 360) + 540) % 360 - 180;
                    return Math.abs(error) < turnToleranceDeg;
                }).raceWith(waitMs(turnTimeoutMs))
        );
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

    // Turret is already tracking; wait for spin-up, then same order as tele: open gate, then feed everything through
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
                instant(() -> robot.intake.setPower(shootFeedPower)),
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


 */