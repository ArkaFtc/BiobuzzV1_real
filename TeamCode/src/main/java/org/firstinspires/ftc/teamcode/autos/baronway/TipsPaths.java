package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

public class TipsPaths {
    public final PoseFactory poseFactory;
    public final Follower follower;

    public final Pose start, push,
            shoot1, garden, shoot2,
            preFlower1, flower1,
            preFlower2, flower2,
            parkShoot1control, shootGardencontrol, gardenShootcontrol1, gardenShootcontrol2,
            flowerShootcontrol, shoot2Flowercontrol, flower2shootcontrol;

    public TipsPaths(Follower follower, Alliance alliance){
        this.follower = follower;

        if (alliance == Alliance.BLUE) {
            poseFactory = PoseFactory.degrees();
        } else {
            poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
        }

        start = poseFactory.of(7, 62, 270);
        push = poseFactory.of(8, 100, 270);
        shoot1 = poseFactory.of(59, 16, 90);
        garden = poseFactory.of(7, 8, 180);
        shoot2 = poseFactory.of(59, 125, -90);
        preFlower1 = poseFactory.of(47, 108, 90);
        flower1 = poseFactory.of(47, 124, 90);
        preFlower2 = poseFactory.of(22, 47, -180);
        flower2 = poseFactory.of(16, 47, -180);
        parkShoot1control = poseFactory.of(16.5, 33.5, -180);
        shootGardencontrol = poseFactory.of(46, 9, -180);
        gardenShootcontrol1 = poseFactory.of(62, 20.25, -180);
        gardenShootcontrol2 = poseFactory.of(61.5, 4.7, -180);
        flowerShootcontrol = poseFactory.of(54.8, 99.1, -180);
        shoot2Flowercontrol = poseFactory.of(26.7, 112.8, -180);
        flower2shootcontrol = poseFactory.of(44.7, 37.67, -180);
    }

    public Pose start() {
        return start;
    }

    public CommandBuilder spikeIntake() {
        return follow(follower, Paths.line(start, spikeIntake).constant(spikeIntake));
    }

    public CommandBuilder spike1Score() {
        return follow(follower, Paths.curve(spikeIntake, spikeScoreControl1, spikeScore).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(spikeScoreSegment1Heading)).until(1, Interpolator.facingPoint(spikeScoreSegment2Target))));
    }

    public CommandBuilder flower1Intake() {
        return follow(follower, Paths.line(spikeScore, flower1Intake).constant(flower1Intake));
    }

    public CommandBuilder flower2Intake() {
        return follow(follower, Paths.curve(flower1Intake, flower2IntakeControl1, flower2IntakeControl2, flower2Intake).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(flower2IntakeSegment1Heading)).until(1, Interpolator.facingPoint(flower2IntakeSegment2Target))));
    }

    public CommandBuilder sweepIntake() {
        return follow(follower, Paths.curve(flower2Intake, sweepIntakeControl1, sweepIntakeControl2, sweepIntake).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(sweepIntakeSegment1Heading)).until(0.3, Interpolator.linear(sweepIntakeSegment2Start, sweepIntakeSegment2End)).until(1, Interpolator.tangent)));
    }

    public CommandBuilder park() {
        return follow(follower, Paths.curve(sweepIntake, parkControl1, park).heading(Interpolator.piecewise().until(0.85, Interpolator.tangent).until(1, Interpolator.constant(parkSegment2Heading))));
    }
}