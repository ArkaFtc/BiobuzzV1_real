package org.firstinspires.ftc.teamcode.autos.baronway;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.utils.Alliance;

public class TipsNoTurretPaths {

    // Points below are written for red. Blue is the same field rotated 180 deg about the center
    // (same symmetry as the tele relocalize poses): (x, y, h) -> (2c - x, 2c - y, h + 180)
    public static double fieldCenter = 72;

    private final Pose start;
    private final Pose push;
    private final Pose pushToShoot;
    private final Pose pushToShootControl1;
    private final Pose intakeGarden;
    private final Pose intakeGardenControl1;
    private final Pose gardenToShoot2;
    // Turned in place after gardenToShoot2 so the back-facing turret can reach the goal
    private final Pose shoot2Turned;
    private final Pose gardenToShoot2Control1;
    private final Pose gardenToShoot2Control2;
    private final Pose toFlower1;
    private final Pose flower1;
    private final Pose flower1ToShoot;
    private final Pose flower1ToShootControl1;
    private final Pose toFlower2;
    private final Pose toFlower2Control1;
    private final Pose flower2;
    private final Pose flower2ToShoot;
    private final Pose flower2ToShootControl1;
    private final Pose park;
    private final Pose parkControl1;

    public TipsNoTurretPaths(Alliance alliance) {
        PoseFactory poseFactory = PoseFactory.degrees();
        if (alliance == Alliance.BLUE) {
            poseFactory = poseFactory.map(p -> new Pose(
                    2 * fieldCenter - p.x(),
                    2 * fieldCenter - p.y(),
                    p.heading() + Math.PI));
        }

        start = poseFactory.of(7, 62, 270);
        push = poseFactory.of(8, 90, 270);
        pushToShoot = poseFactory.of(58, 16, -21.946);
        pushToShootControl1 = poseFactory.of(16.6731, 32.6519, 0);
        intakeGarden = poseFactory.of(7, 4, -178.5312);
        intakeGardenControl1 = poseFactory.of(46, 7, 0);
        gardenToShoot2 = poseFactory.of(60, 125, -89.2439);
        shoot2Turned = poseFactory.of(60, 125, 90);
        gardenToShoot2Control1 = poseFactory.of(61.9787, 20.2935, 0);
        gardenToShoot2Control2 = poseFactory.of(61.588, 4.6731, 0);
        toFlower1 = poseFactory.of(47, 108, 90);
        flower1 = poseFactory.of(47, 127, 90);
        flower1ToShoot = poseFactory.of(60, 125, 87.026);
        flower1ToShootControl1 = poseFactory.of(54.8287, 99.0907, 0);
        toFlower2 = poseFactory.of(22, 47, -94.0783);
        toFlower2Control1 = poseFactory.of(26.6944, 112.8407, 0);
        flower2 = poseFactory.of(13, 47, 180);
        flower2ToShoot = poseFactory.of(58, 16, 121.5466);
        flower2ToShootControl1 = poseFactory.of(28.75, 11.75, 0);
        park = poseFactory.of(8, 100, 97.2519);
        parkControl1 = poseFactory.of(16.4639, 33.4861, 0);
    }

    public Pose start() {
        return start;
    }

    public Path push() {
        return Paths.line(start, push).constant(push);
    }

    public Path pushToShoot() {
        return Paths.curve(push, pushToShootControl1, pushToShoot).tangent();
    }

    public Path intakeGarden() {
        return Paths.curve(pushToShoot, intakeGardenControl1, intakeGarden).tangent();
    }

    public Path gardenToShoot2() {
        return Paths.curve(intakeGarden, gardenToShoot2Control1, gardenToShoot2Control2, gardenToShoot2).reverseTangent();
    }

    public Pose shoot2Turned() {
        return shoot2Turned;
    }

    public Path toFlower1() {
        // Already at 90 after the shot 2 turn, so no turn needed on the way to flower 1
        return Paths.line(shoot2Turned, toFlower1).constant(toFlower1);
    }

    public Path flower1() {
        return Paths.line(toFlower1, flower1).constant(flower1);
    }

    public Path flower1ToShoot() {
        return Paths.curve(flower1, flower1ToShootControl1, flower1ToShoot).linear(flower1, flower1ToShoot);
    }

    public Path toFlower2() {
        return Paths.curve(flower1ToShoot, toFlower2Control1, toFlower2).tangent();
    }

    public Path flower2() {
        return Paths.line(toFlower2, flower2).tangent();
    }

    public Path flower2ToShoot() {
        return Paths.curve(flower2, flower2ToShootControl1, flower2ToShoot).reverseTangent();
    }

    public Path park() {
        return Paths.curve(flower2ToShoot, parkControl1, park).tangent();
    }
}
