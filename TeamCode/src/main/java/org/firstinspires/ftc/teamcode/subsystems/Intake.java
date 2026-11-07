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

    // Low pass filter on beam breaks 2, 3 and 4 (shared settings).
    // beamFilterTime: how long a steady change takes to flip the filtered output (seconds)
    // beamThreshold: filtered value needed to turn on; it turns off below (1 - beamThreshold)
    public static double beamFilterTime = 0.2;
    public static double beamThreshold = 0.7;
    private final BeamFilter back2Filter = new BeamFilter();
    private final BeamFilter ball3Filter = new BeamFilter();
    private final BeamFilter ball4Filter = new BeamFilter();

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
        back2Filter.update(ball2());
        ball3Filter.update(ball3());
        ball4Filter.update(ball4());
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
    // Filtered beam breaks. Short blips only nudge the filtered value, so each keeps its
    // previous state unless the beam break changes for about beamFilterTime seconds.
    public boolean hasBack2(){
        return back2Filter.on;
    }
    public boolean hasBall3(){
        return ball3Filter.on;
    }
    public boolean hasBall4(){
        return ball4Filter.on;
    }

    // Filtered values between 0 and 1, useful for telemetry/graphing
    public double getBack2Value(){
        return back2Filter.value;
    }
    public double getBall3Value(){
        return ball3Filter.value;
    }
    public double getBall4Value(){
        return ball4Filter.value;
    }

    private static class BeamFilter {
        double value = 0;       // 0 = clear, 1 = ball
        boolean on = false;
        private long lastTime = -1;

        void update(boolean rawOn){
            long now = System.nanoTime();
            if (lastTime < 0) {
                lastTime = now;
                return;
            }
            double dt = (now - lastTime) / 1e9;
            lastTime = now;

            // Pick the time constant so a steady change crosses the threshold in beamFilterTime
            double tau = beamFilterTime / Math.log(1.0 / (1.0 - beamThreshold));
            double alpha = dt / (tau + dt);
            double raw = rawOn ? 1.0 : 0.0;
            value += alpha * (raw - value);

            // Hysteresis: separate on/off thresholds so it doesn't flicker around one value
            if (value >= beamThreshold) on = true;
            else if (value <= 1.0 - beamThreshold) on = false;
        }
    }
    // Back 3 slots full, using the filtered beam breaks
    public boolean all4(){
        return hasBack2() && hasBall3() && hasBall4();
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