package org.firstinspires.ftc.teamcode.autos.baronway;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class RedTipsNoTurretPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(7, 62, 270);
    private final Pose push = poseFactory.of(8, 100, 270);
    private final Pose pushToShoot = poseFactory.of(58, 16, -21.946);
    private final Pose pushToShootControl1 = poseFactory.of(16.6731, 32.6519, 0);
    private final Pose turnToShoot1Start = poseFactory.of(58, 16, -22);
    private final Pose turnToShoot1 = poseFactory.of(58, 16, 90);
    private final Pose intakeGarden = poseFactory.of(7, 8, -178.5312);
    private final Pose intakeGardenControl1 = poseFactory.of(46, 9, 0);
    private final Pose gardenToShoot2 = poseFactory.of(60, 125, -89.2439);
    private final Pose gardenToShoot2Control1 = poseFactory.of(61.9787, 20.2935, 0);
    private final Pose gardenToShoot2Control2 = poseFactory.of(61.588, 4.6731, 0);
    private final Pose toFlower1Start = poseFactory.of(60, 125, -90);
    private final Pose toFlower1 = poseFactory.of(47, 108, 90);
    private final Pose flower1 = poseFactory.of(47, 130, 90);
    private final Pose flower1ToShoot = poseFactory.of(60, 125, -101.2875);
    private final Pose flower1ToShootControl1 = poseFactory.of(54.8287, 99.0907, 0);
    private final Pose toFlower2 = poseFactory.of(22, 47, -94.0783);
    private final Pose toFlower2Control1 = poseFactory.of(26.6944, 112.8407, 0);
    private final Pose flower2 = poseFactory.of(10, 47, 180);
    private final Pose flower2ToShoot = poseFactory.of(58, 16, 121.5466);
    private final Pose flower2ToShootControl1 = poseFactory.of(44.6944, 37.6731, 0);
    private final Pose park = poseFactory.of(8, 100, 97.2519);
    private final Pose parkControl1 = poseFactory.of(16.4639, 33.4861, 0);

    public Pose start() {
        return start;
    }

    public Path push() {
        return Paths.line(start, push).constant(push);
    }

    public Path pushToShoot() {
        return Paths.curve(push, pushToShootControl1, pushToShoot).tangent();
    }

    public Path turnToShoot1() {
        return Paths.line(turnToShoot1Start, turnToShoot1).linear(turnToShoot1Start, turnToShoot1);
    }

    public Path intakeGarden() {
        return Paths.curve(turnToShoot1, intakeGardenControl1, intakeGarden).tangent();
    }

    public Path gardenToShoot2() {
        return Paths.curve(intakeGarden, gardenToShoot2Control1, gardenToShoot2Control2, gardenToShoot2).reverseTangent();
    }

    public Path toFlower1() {
        return Paths.line(toFlower1Start, toFlower1).linear(toFlower1Start, toFlower1);
    }

    public Path flower1() {
        return Paths.line(toFlower1, flower1).constant(flower1);
    }

    public Path flower1ToShoot() {
        return Paths.curve(flower1, flower1ToShootControl1, flower1ToShoot).reverseTangent();
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
