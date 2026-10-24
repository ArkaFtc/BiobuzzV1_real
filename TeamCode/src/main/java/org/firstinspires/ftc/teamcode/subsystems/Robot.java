package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import java.util.List;

import static org.firstinspires.ftc.teamcode.pedro.Constants.create;

public class Robot {
    private List<LynxModule> hubs;
    public Intake intake;
    public FlowerMech flowermech;
    public Shooter shooter;
    public Follower follower;
    public static Pose savedPose = Pose.zero();
    public Alliance alliance;
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        flowermech = new FlowerMech(hardwareMap);
        follower = create(hardwareMap);

        loop.reset();
    }

    public void update() {
        loops++;

        if (loops > 10) {
            double now = loop.milliseconds();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        clearCache();
        intake.update();
        flowermech.update();
        shooter.update();
        follower.update();
        savedPose = follower.pose();
    }
    public Pose pose() {
        return follower.pose();
    }

    public void pose(Pose pose) {
        follower.setPose(pose);
    }
    public void clearCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    public void saveEnd() {
        savedPose = follower.pose();
    }

    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
