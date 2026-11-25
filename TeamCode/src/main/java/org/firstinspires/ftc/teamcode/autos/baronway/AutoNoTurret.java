package org.firstinspires.ftc.teamcode.autos.baronway;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.function.BooleanSupplier;

// Shared auto; redAutoNoTurret / blueAutoNoTurret pick the alliance (blue paths are mirrored in TipsNoTurretPaths)
// Two hive checks pick the route (same idea as "AprilTag Seen (webcam)": can we see the hive's tags, yes or no):
//   check 1 at toShoot2 (top spot), check 2 at the next bottom spot (toShoot4 for G1, toShoot5 for B1).
//   Tags seen = good (shoot here). No tags = bad (hive tipped, drive to the other spot and shoot there).
@Configurable
public abstract class AutoNoTurret extends CommandOpMode {
    public static double pathTimeoutMs = 4000;
    public static double spinUpTimeoutMs = 1500;
    public static double velocityTolerance = 40;
    public static double gateOpenMs = 100;
    public static double shootMs = 700;
    public static double shootFeedPower = 0.8;
    // The flower arm + dropdown go down BEFORE the robot starts driving to a flower; wait this long for them to finish moving
    public static double flowerArmSettleMs = 500;
    // Flower path completion that counts as "at the flower"; the arm-down timer starts here
    public static double flowerArriveCompletion = 0.9;
    // How long the flower arm stays down at the flower before lifting and moving on
    public static double flowerArmDownMs = 1000;
    // Speed cap (in/s) while driving into a flower (same idea as gardenIntakeSpeed)
    public static double flowerIntakeSpeed = 30;
    // Path completion on the way back from a flower at which the intake dropdown goes down
    public static double flower1ReturnDropCompletion = 0.1;
    public static double flower2ReturnDropCompletion = 0.5;
    public static double turnToleranceDeg = 5;
    public static double turnTimeoutMs = 1500;
    public static double idleFrontPower = 0.6;
    // Speed cap (in/s) for the garden intake path; max achievable forward is ~71
    public static double gardenIntakeSpeed = 30;
    // Preload intake path: everything after this point along the curve (0-1) is capped at preloadEndSpeed (in/s)
    public static double preloadSlowFromT = 0.75;
    public static double preloadEndSpeed = 20;

    // Shooter settings: first shot (preloads) vs every other shot
    public static double firstShotVelocity = 1600;
    public static double firstShotHood = 0.5;
    public static double shotVelocity = 1500;
    public static double shotHood = 0.63;

    // ---- Hive tip check (Logitech C270 webcam, "Webcam 1"; the SDK has a built-in C270 calibration at 640x480) ----
    // How long the robot stops and looks for the hive's tags at a check spot (moves on sooner if it sees one)
    public static double tipCheckMs = 1000;
    // false = check 1 looks at the upper basket's tags, check 2 at the lower basket's. true = any tag on our hive counts
    public static boolean checkWholeHive = false;
    static final String WEBCAM_NAME = "Webcam 1";
    // Tag IDs per basket (same as AprilTagSeenWebcamOpMode)
    static final int[] RED_UPPER_TAGS = {30, 31, 32, 33};    // B
    static final int[] RED_LOWER_TAGS = {34, 35, 36, 37};    // A
    static final int[] BLUE_UPPER_TAGS = {42, 43, 44, 45};   // Y
    static final int[] BLUE_LOWER_TAGS = {38, 39, 40, 41};   // X

    final Alliance alliance;
    TipsNoTurretPaths paths;
    Robot robot;
    boolean intaking = false;

    VisionPortal portal;
    AprilTagProcessor aprilTag;
    boolean hive1Good = true;   // result of check 1 (true = tags seen)
    boolean hive2Good = true;   // result of check 2
    boolean tagSeenNow = false;
    String routeName = "checking...";

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

        // Webcam: SDK 12 single-tag library (IDs 0-100, 3.25 in) so every tag keeps its own ID
        AprilTagLibrary.Builder lib = new AprilTagLibrary.Builder();
        for (int id = 0; id <= 100; id++) {
            lib.addTag(id, "Tag " + id, 3.25, DistanceUnit.INCH);
        }
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(lib.build())
                .build();
        aprilTag.setDecimation(2);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();
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
                            robot.shooter.setTargetVelocity(firstShotVelocity);
                            robot.shooter.setHood(firstShotHood);
                        }),

                        // Preloads: shoot all 4 from the start spot
                        shoot(),
                        // Every shot after the first
                        instant(() -> {
                            robot.shooter.setTargetVelocity(shotVelocity);
                            robot.shooter.setHood(shotHood);
                        }),

                        // Garden (intake stays on the whole auto)
                        intakeOn(),
                        drive(paths.gardenIntake().with(Constants.foresightConfig.maxPathSpeed.at(gardenIntakeSpeed))),
                        drive(paths.toShoot2()),
                        turnTo(paths.toShoot2Turned()),

                        // Hive check 1 (top spot, upper basket)
                        hiveCheck(1),

                        // ===== GOOD 1: shoot here, flower 1, preload, bottom spot =====
                        onlyIf(() -> hive1Good, sequential(
                                shoot(),
                                intakeOn(),
                                flowerDrive(paths.flower1IntakeG1G2G1B2()),
                                flowerReturn(paths.toShoot3(), flower1ReturnDropCompletion),
                                shoot(),
                                intakeOn(),
                                drive(paths.preloadIntake(preloadSlowFromT, preloadEndSpeed)),
                                drive(paths.toShoot4G1G2G1B2()),

                                // Hive check 2 (bottom spot, lower basket)
                                hiveCheck(2),

                                // G1G2: shoot here, then flower 2
                                onlyIf(() -> hive2Good, sequential(
                                        shoot(),
                                        intakeOn(),
                                        flowerDrive(paths.flower2IntakeG1G2())
                                )),
                                // G1B2: go back to the top spot to shoot, then flower 2
                                onlyIf(() -> !hive2Good, sequential(
                                        drive(paths.return1ShootG1B2()),
                                        turnTo(paths.return1ShootG1B2Turned()),
                                        shoot(),
                                        intakeOn(),
                                        flowerDrive(paths.flower2IntakeG1B2())
                                )),

                                flowerReturn(paths.toShoot5(), flower2ReturnDropCompletion),
                                shoot(),
                                intakeOn(),
                                drive(paths.parkG1G2G1B2B1G2())
                        )),

                        // ===== BAD 1: drive to the bottom spot and shoot there =====
                        onlyIf(() -> !hive1Good, sequential(
                                drive(paths.return1ShootB1G2B1B2()),
                                turnTo(paths.return1ShootB1G2B1B2Turned()),
                                shoot(),
                                intakeOn(),
                                flowerDrive(paths.flower1IntakeB1G2B1B2()),
                                flowerReturn(paths.toShoot3(), flower1ReturnDropCompletion),
                                shoot(),
                                intakeOn(),
                                drive(paths.preloadIntake(preloadSlowFromT, preloadEndSpeed)),
                                drive(paths.toShoot4B1G2B1B2()),
                                shoot(),
                                intakeOn(),
                                flowerDrive(paths.flower2IntakeB1G2B1B2()),
                                flowerReturn(paths.toShoot5(), flower2ReturnDropCompletion),

                                // Hive check 2 (bottom spot, lower basket)
                                hiveCheck(2),

                                // B1G2: shoot here and park
                                onlyIf(() -> hive2Good, sequential(
                                        shoot(),
                                        intakeOn(),
                                        drive(paths.parkG1G2G1B2B1G2())
                                )),
                                // B1B2: go to the top spot to shoot, then park
                                onlyIf(() -> !hive2Good, sequential(
                                        intakeOn(),
                                        drive(paths.return2ShootB1B2()),
                                        turnTo(paths.return2ShootB1B2Turned()),
                                        shoot(),
                                        intakeOn(),
                                        drive(paths.parkB1B2())
                                ))
                        ))
                ),
                infinite(() -> {
                    robot.update();
                    // Turret always tracks the closest goal; flywheel velocity/hood are not set here
                    robot.shooter.aimTurret(robot.pose(), alliance);
                    telemetry.addData("turret angle (deg)", robot.shooter.getAngleDistance(robot.pose(), alliance)[0]);
                    if (intaking)
                        robot.intake.setTransferPower(robot.intake.hasBack2() ? 0 : 1);

                    telemetry.addData("route", routeName);
                    telemetry.addData("hive tag seen now", tagSeenNow);
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
        telemetry.addData("Camera", portal.getCameraState());
        telemetry.addData("upper basket tags seen", anySeen(1));
        telemetry.addData("lower basket tags seen", anySeen(2));
        telemetry.addData("pose", robot.pose());
        telemetry.addData("turret angle (deg)", robot.shooter.getAngleDistance(robot.pose(), alliance)[0]);
        telemetry.addData("shooter target", robot.shooter.getTargetVelo());
        telemetry.addData("shooter velocity", robot.shooter.getCurrentVelocity());
        telemetry.update();
    }

    CommandBuilder drive(Path path) {
        return follow(robot.follower, path).raceWith(waitMs(pathTimeoutMs));
    }

    // Lowers the flower arm first, waits for it to get down, then drives the flower path; moves on flowerArmDownMs after reaching the flower
    // instead of waiting for the path to finish (which can stall against the wall until the timeout)
    CommandBuilder flowerDrive(Path path) {
        return sequential(
                instant(() -> {
                    robot.flowermech.setFlowerArm(true);
                    robot.intake.setDropDownMid();
                }),
                waitMs(flowerArmSettleMs),
                instant(() -> robot.follower.follow(path.with(Constants.foresightConfig.maxPathSpeed.at(flowerIntakeSpeed)))),
                // Arm is down early; start the timer once the robot is at the flower so long paths still get the full time
                waitUntil(() -> robot.follower.completion() > flowerArriveCompletion)
                        .raceWith(waitMs(pathTimeoutMs)),
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

    // ---- Hive tip check ----

    // Looks for the hive's tags for up to tipCheckMs. Seen at any moment = good, never seen = bad (tipped)
    CommandBuilder hiveCheck(int check) {
        return sequential(
                instant(() -> tagSeenNow = false),
                waitUntil(() -> {
                    tagSeenNow = anySeen(check);
                    return tagSeenNow;
                }).raceWith(waitMs(tipCheckMs)),
                instant(() -> {
                    if (check == 1) hive1Good = tagSeenNow;
                    else hive2Good = tagSeenNow;
                    routeName = (hive1Good ? "G1" : "B1") + (check == 2 ? (hive2Good ? "G2" : "B2") : "");
                })
        );
    }

    // Runs cmd only if cond is true when this step is reached; otherwise skips it without starting it
    CommandBuilder onlyIf(BooleanSupplier cond, CommandBuilder cmd) {
        return waitUntil(() -> !cond.getAsBoolean())
                .raceWith(sequential(waitUntil(() -> cond.getAsBoolean()), cmd));
    }

    // True if the webcam sees any of the tags for this check (1 = upper basket, 2 = lower basket)
    boolean anySeen(int check) {
        boolean red = alliance != Alliance.BLUE;
        int[] upper = red ? RED_UPPER_TAGS : BLUE_UPPER_TAGS;
        int[] lower = red ? RED_LOWER_TAGS : BLUE_LOWER_TAGS;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            // SDK 12: only a single-tag detection has an id
            if (!(d instanceof AprilTagSingleDetection)) continue;
            int id = ((AprilTagSingleDetection) d).id;
            if ((checkWholeHive || check == 1) && contains(upper, id)) return true;
            if ((checkWholeHive || check == 2) && contains(lower, id)) return true;
        }
        return false;
    }

    static boolean contains(int[] ids, int id) {
        for (int i : ids) if (i == id) return true;
        return false;
    }

    @Override
    public void stop() {
        robot.saveEnd();
        if (portal != null) portal.close();
        super.stop();
    }
}
