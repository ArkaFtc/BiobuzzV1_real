package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.CachedServo;

@Config

public class FlowerMech {
    CachedServo flowerServo;


    public static double flowerDownPos = 0.38;
    public static double flowerIdlePos = 0;

    public boolean isDown = false;

    public FlowerMech(HardwareMap hardwareMap) {
        flowerServo = new CachedServo(hardwareMap, "flowerServo");
    }

    public void update() {
        flowerServo.update();
    }

    public void setFlowerArm(boolean down){
        if (down){
            flowerServo.setPosition(flowerDownPos);
        } else {
            flowerServo.setPosition(flowerIdlePos);
        }
        isDown = down;
    }
}