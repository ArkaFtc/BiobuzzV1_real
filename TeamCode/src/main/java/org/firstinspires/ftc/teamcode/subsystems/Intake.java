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

    // Low pass filter on beam break 2.
    // back2FilterTime: how long a steady change takes to flip hasBack2() (seconds)
    // back2Threshold: filtered value needed to turn on; it turns off below (1 - back2Threshold)
    public static double back2FilterTime = 0.2;
    public static double back2Threshold = 0.7;
    private double back2Value = 0;          // filtered beam break, 0 = clear, 1 = ball
    private boolean back2Filtered = false;
    private long back2LastTime = -1;

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
        front.set(-power);
    }

    public void setTransferPower(double power){
        transfer.set(-power);
    }


    public void update() {
        updateBack2Filter();
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
    public double getFrontCurrent(){
        return front.getCurrentDraw();
    }
    public double getTransferCurrent(){
        return transfer.getCurrentDraw();
    }

    // Halfway between fully down and idle (up); follows both if they get retuned
    public static double getDropMidPos(){
        return (dropDownPos + dropUpPos) / 2.0;
    }
    public void setDropDownMid(){
        dropDown.setPosition(getDropMidPos());
    }
    // Filtered beam break 2. Short blips only nudge the filtered value, so this keeps its
    // previous state unless beam break 2 changes for about back2FilterTime seconds.
    public boolean hasBack2(){
        return back2Filtered;
    }

    // Filtered value between 0 and 1, useful for telemetry/graphing
    public double getBack2Value(){
        return back2Value;
    }

    private void updateBack2Filter(){
        long now = System.nanoTime();
        if (back2LastTime < 0) {
            back2LastTime = now;
            return;
        }
        double dt = (now - back2LastTime) / 1e9;
        back2LastTime = now;

        // Pick the time constant so a steady change crosses the threshold in back2FilterTime
        double tau = back2FilterTime / Math.log(1.0 / (1.0 - back2Threshold));
        double alpha = dt / (tau + dt);
        double raw = ball2() ? 1.0 : 0.0;
        back2Value += alpha * (raw - back2Value);

        // Hysteresis: separate on/off thresholds so it doesn't flicker around one value
        if (back2Value >= back2Threshold) back2Filtered = true;
        else if (back2Value <= 1.0 - back2Threshold) back2Filtered = false;
    }
    public boolean all4(){
        if (ball2() &&ball3() &&ball4()){
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