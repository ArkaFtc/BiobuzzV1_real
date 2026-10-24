package org.firstinspires.ftc.teamcode.autos.baronway;

import org.firstinspires.ftc.teamcode.autos.TipsPaths;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;

public class hopefullytips extends CommandOpMode {
    final Alliance alliance;
    TipsPaths paths;
    Robot robot;

    public hopefullytips(Alliance alliance) {
        this.alliance = alliance;
    }

    public void init() {
        robot = new Robot(hardwareMap, alliance);
        paths = new TipsPaths(robot.follower, alliance);

        robot.pose(paths.start());

        schedule(
            robot.intake.setIntakePower(0.4);
            robot.intake.setPower(0);
        );
    }
}