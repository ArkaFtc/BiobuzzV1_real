package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.utils.CachedMotor;
import org.firstinspires.ftc.teamcode.utils.CachedServo;

@Configurable
public class Intake {
    CachedMotor front;
    CachedMotor transfer;
    CachedServo dropDown;

    TouchSensor beamBreakball1;
    TouchSensor beamBreakball2;
    TouchSensor beamBreakball3;
    TouchSensor beamBreakball4;

    public static double currentLimit = 4;
    public static double dropDownPos = 0.43;
    public static double dropUpPos = 0.67;
    public boolean gateOpened = false;
    public double intakePower;

    public Intake(HardwareMap hardwareMap) {
        dropDown = new CachedServo(hardwareMap, "dropDown");
        front = new CachedMotor(hardwareMap, "frontIntake");
        transfer = new CachedMotor(hardwareMap, "transferIntake");
        beamBreakball1 = hardwareMap.touchSensor.get("break1");
        beamBreakball2 = hardwareMap.touchSensor.get("break2");
        beamBreakball3 = hardwareMap.touchSensor.get("break3");
        beamBreakball4 = hardwareMap.touchSensor.get("break4");
        front.setCurrentAlert(currentLimit);
        transfer.setCurrentAlert(currentLimit);
        update();
    }

    public void setPower(double power) {
        intakePower = power;
        setIntakePower(power);
        setTransferPower(power);
    }
    public void setIntakePower(double power){
        if (front.isOverCurrent()){
            double current = front.getCurrentDraw();
            if (current != 0) {
                power = power * currentLimit / current;
            }
        }
        front.set(power);
    }

    public void setTransferPower(double power){
        transfer.set(power);
    }


    public void update() {
        front.update();
        transfer.update();
        dropDown.update();
    }
    public void setDropDown(boolean drop){
        if (drop){
            dropDown.setPosition(dropDownPos);
        } else {
            dropDown.setPosition(dropUpPos);
        }
    }
    public boolean all4(){
        if (ball1() && ball2() &&ball3() &&ball4()){
            return true;
        }
        return false;
    }
    public boolean ball1(){
        return beamBreakball1.isPressed();
    }
    public boolean ball2(){
        return beamBreakball2.isPressed();
    }
    public boolean ball3(){
        return beamBreakball3.isPressed();
    }
    public boolean ball4(){
        return beamBreakball4.isPressed();
    }
}