package org.firstinspires.ftc.teamcode.autos.baronway;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.Alliance;

// Paths for AutoNoTurret, from the four .pp.json files (good1good2, good1bad2, bad1good2, bad1bad2).
// Points are drawn for RED. For BLUE every point is mirrored across the middle of the field (x -> 144 - x, heading -> 180 - heading).
// Suffixes say which cases use a point/path: G1G2, G1B2, B1G2, B1B2 (good/bad hive check 1, good/bad hive check 2).
public class TipsNoTurretPaths {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final boolean blue;

    private final Pose start;
    private final Pose gardenIntake;
    private final Pose toShoot2, toShoot2Control1, toShoot2Control2;
    private final Pose flower1Intake, flower1IntakeControl1B1G2B1B2;
    private final Pose toShoot3;
    private final Pose preloadIntake, preloadIntakeControl1, preloadIntakeControl2;
    private final Pose toShoot4G1G2G1B2, toShoot4G1G2G1B2Control1, toShoot4G1G2G1B2Control2;
    private final Pose toShoot4B1G2B1B2, toShoot4B1G2B1B2Control1, toShoot4B1G2B1B2Control2;
    private final Pose return1ShootG1B2, return1ShootG1B2Control1, return1ShootG1B2Control2;
    private final Pose return1ShootB1G2B1B2, return1ShootB1G2B1B2Control1, return1ShootB1G2B1B2Control2;
    private final Pose flower2Intake;
    private final Pose flower2IntakeControl1G1G2, flower2IntakeControl2G1G2;
    private final Pose flower2IntakeControl1G1B2B1G2B1B2, flower2IntakeControl2G1B2B1G2B1B2;
    private final Pose toShoot5, toShoot5Control1, toShoot5Segment2Start;
    private final Pose return2ShootB1B2, return2ShootB1B2Control1, return2ShootB1B2Control2;
    private final Pose park, parkControl1;

    public TipsNoTurretPaths(Alliance alliance) {
        blue = alliance == Alliance.BLUE;

        start = p(30, 8, 180);
        gardenIntake = p(8.4964, 7, 180);
        toShoot2 = p(47, 120, 90);
        toShoot2Control1 = p(83.0427, 15.2898, 0);
        toShoot2Control2 = p(64.382, 116.232, 0);
        flower1Intake = p(47.0147, 130.0272, 90);
        flower1IntakeControl1B1G2B1B2 = p(17.9707, 68.6686, 0);
        toShoot3 = p(47, 116, 90);
        preloadIntake = p(6, 97.5, -90);
        preloadIntakeControl1 = p(8, 113, 0);
        preloadIntakeControl2 = p(8.49, 129.4294, 0);
        toShoot4G1G2G1B2 = p(52.3842, 18.872, 180);
        toShoot4G1G2G1B2Control1 = p(31.9209, 141.5, 0);
        toShoot4G1G2G1B2Control2 = p(14.4415, 18.9527, 0);
        toShoot4B1G2B1B2 = p(47, 116, 180);
        toShoot4B1G2B1B2Control1 = p(8.49, 129.43, 0);
        toShoot4B1G2B1B2Control2 = p(8, 113, 0);
        return1ShootG1B2 = p(46.9777, 120.1721, 180);
        return1ShootG1B2Control1 = p(75.4749, 79.6057, 0);
        return1ShootG1B2Control2 = p(55.1096, 117.3515, 0);
        return1ShootB1G2B1B2 = p(44, 19, 90);
        return1ShootB1G2B1B2Control1 = p(68.46, 101.25, 0);
        return1ShootB1G2B1B2Control2 = p(66.89, 49.883, 0);
        flower2Intake = p(12, 47, 180);
        flower2IntakeControl1G1G2 = p(34, 46, 0);
        flower2IntakeControl2G1G2 = p(27, 47, 0);
        flower2IntakeControl1G1B2B1G2B1B2 = p(24.54, 118.15, 0);
        flower2IntakeControl2G1B2B1G2B1B2 = p(50.34, 44.69, 0);
        toShoot5 = p(40, 19, 110);
        toShoot5Control1 = p(44, 50, 0);
        toShoot5Segment2Start = p(44, 19, 110.4011);
        return2ShootB1B2 = p(47.0477, 120.0562, 90);
        return2ShootB1B2Control1 = p(80.2369, 73.9847, 0);
        return2ShootB1B2Control2 = p(53.0607, 118.474, 0);
        park = p(10, 90, -90);
        parkControl1 = p(10, 42, 0);
    }

    // Red point as drawn; blue = mirrored left/right
    private Pose p(double x, double y, double headingDeg) {
        return blue ? poseFactory.of(144 - x, y, 180 - headingDeg) : poseFactory.of(x, y, headingDeg);
    }

    public Pose start() {
        return start;
    }

    // ---- All cases ----
    public Path gardenIntake() {
        return Paths.line(start, gardenIntake).constant(gardenIntake);
    }

    // Hive check 1 happens here (top shooting spot)
    public Path toShoot2() {
        return Paths.curve(gardenIntake, toShoot2Control1, toShoot2Control2, toShoot2).reverseTangent();
    }

    // The path file turns to 90 at the end of toShoot2
    public Pose toShoot2Turned() {
        return toShoot2;
    }

    // ---- Good 1 (G1G2, G1B2) ----
    public Path flower1IntakeG1G2G1B2() {
        return Paths.line(toShoot2, flower1Intake).constant(flower1Intake);
    }

    // ---- Bad 1 (B1G2, B1B2) ----
    public Path return1ShootB1G2B1B2() {
        return Paths.curve(toShoot2, return1ShootB1G2B1B2Control1, return1ShootB1G2B1B2Control2, return1ShootB1G2B1B2).reverseTangent();
    }

    public Pose return1ShootB1G2B1B2Turned() {
        return return1ShootB1G2B1B2;
    }

    // Holds 90 the whole way so the flower arm faces the flower when it arrives
    public Path flower1IntakeB1G2B1B2() {
        return Paths.curve(return1ShootB1G2B1B2, flower1IntakeControl1B1G2B1B2, flower1Intake).constant(flower1Intake);
    }

    // ---- All cases ----
    public Path toShoot3() {
        return Paths.line(flower1Intake, toShoot3).reverseTangent();
    }

    public Path preloadIntake() {
        return Paths.curve(toShoot3, preloadIntakeControl1, preloadIntakeControl2, preloadIntake).tangent();
    }

    // Same curve as preloadIntake(), split at splitT (0-1 along the curve) so the end can be driven
    // slower: everything after splitT is capped at endSpeed (in/s). Both pieces trace the exact same curve.
    public Path preloadIntake(double splitT, double endSpeed) {
        Pose[] pts = {toShoot3, preloadIntakeControl1, preloadIntakeControl2, preloadIntake};
        // De Casteljau split of the cubic bezier
        Pose a = lerp(pts[0], pts[1], splitT), b = lerp(pts[1], pts[2], splitT), c = lerp(pts[2], pts[3], splitT);
        Pose d = lerp(a, b, splitT), e = lerp(b, c, splitT);
        Pose mid = lerp(d, e, splitT);
        Path first = Paths.curve(pts[0], a, d, mid).tangent();
        Path end = Paths.curve(mid, e, c, pts[3]).tangent()
                .with(Constants.foresightConfig.maxPathSpeed.at(endSpeed));
        return Paths.path(first, end);
    }

    private static Pose lerp(Pose p, Pose q, double t) {
        return new Pose(p.x() + (q.x() - p.x()) * t, p.y() + (q.y() - p.y()) * t);
    }

    // Hive check 2 happens here for G1 cases (bottom shooting spot)
    public Path toShoot4G1G2G1B2() {
        return Paths.curve(preloadIntake, toShoot4G1G2G1B2Control1, toShoot4G1G2G1B2Control2, toShoot4G1G2G1B2).reverseTangent();
    }

    public Path toShoot4B1G2B1B2() {
        return Paths.curve(preloadIntake, toShoot4B1G2B1B2Control1, toShoot4B1G2B1B2Control2, toShoot4B1G2B1B2).reverseTangent();
    }

    // ---- G1B2 only ----
    public Path return1ShootG1B2() {
        return Paths.curve(toShoot4G1G2G1B2, return1ShootG1B2Control1, return1ShootG1B2Control2, return1ShootG1B2).tangent();
    }

    public Pose return1ShootG1B2Turned() {
        return return1ShootG1B2;
    }

    // ---- flower2Intake ----
    public Path flower2IntakeG1G2() {
        return Paths.curve(toShoot4G1G2G1B2, flower2IntakeControl1G1G2, flower2IntakeControl2G1G2, flower2Intake).tangent();
    }

    public Path flower2IntakeG1B2() {
        return Paths.curve(return1ShootG1B2, flower2IntakeControl1G1B2B1G2B1B2, flower2IntakeControl2G1B2B1G2B1B2, flower2Intake).tangent();
    }

    public Path flower2IntakeB1G2B1B2() {
        return Paths.curve(toShoot4B1G2B1B2, flower2IntakeControl1G1B2B1G2B1B2, flower2IntakeControl2G1B2B1G2B1B2, flower2Intake).tangent();
    }

    // Hive check 2 happens here for B1 cases (bottom shooting spot)
    public Path toShoot5() {
        // Reverse tangent for most of the path, then blend the heading to toShoot5's heading at the end
        return Paths.curve(flower2Intake, toShoot5Control1, toShoot5).heading(Interpolator.piecewise()
                .until(0.6993, Interpolator.tangent.reverse())
                .until(1, Interpolator.linear(toShoot5Segment2Start, toShoot5)));
    }

    // ---- B1B2 only ----
    public Path return2ShootB1B2() {
        return Paths.curve(toShoot5, return2ShootB1B2Control1, return2ShootB1B2Control2, return2ShootB1B2).tangent();
    }

    public Pose return2ShootB1B2Turned() {
        return return2ShootB1B2;
    }

    // ---- park ----
    public Path parkG1G2G1B2B1G2() {
        return Paths.curve(toShoot5, parkControl1, park).reverseTangent();
    }

    public Path parkB1B2() {
        return Paths.curve(return2ShootB1B2, parkControl1, park).reverseTangent();
    }
}
