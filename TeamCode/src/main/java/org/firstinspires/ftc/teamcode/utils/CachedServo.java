package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

/**
 * An extended Servo wrapper class which implements utility features such as
 * caching to reduce loop times and custom angle ranges.
 */
public class CachedServo {
    private final Servo servo;
    private final String id;
    private double min = 0.0;
    private double max = 1.0;
    private double cachingTolerance = 0.005;

    private double curr_raw_pos = -1;
    private double prev_raw_pos = -1;


    /**
     * The main constructor for the ServoEx object.
     * @param hwMap hardwareMap
     * @param id the ID of the servo as configured
     * @param min the minimum angle of the servo in the specified angle unit (when the servo is set to 0)
     * @param max the maximum angle of the servo in the specified angle unit (when the servo is set to 1)
     */
    public CachedServo(HardwareMap hwMap, String id, double min, double max) {
        this.servo = hwMap.get(Servo.class, id);
        this.id = id;
        this.min = min;
        this.max = max;
    }
    /**
     * @param hwMap hardwareMap
     * @param id the ID of the servo as configured
     */
    public CachedServo(HardwareMap hwMap, String id) {
        this(hwMap, id, 0.0, 1.0);
    }

    /**
     * @param output the raw position (or angle if range or max + min were defined in constructor) the servo should be set to
     */
    public void setPosition(double output) {
        curr_raw_pos = (output - min) / (max - min);
    }

    /**
     * Updates the servo position to the specified angle in degrees.
     * Run this every loop
     */
    public void update(){
        if (Math.abs(curr_raw_pos - prev_raw_pos) > cachingTolerance) {
            servo.setPosition(curr_raw_pos);
            prev_raw_pos = curr_raw_pos;
        }
    }

    public double get() {
        return getRawPosition() * (max - min) + min;
    }

    /**
     * @return the raw position of the servo between 0 and 1
     */
    private double getRawPosition() {
        return curr_raw_pos;
    }

    /**
     * @param inverted whether the servo should be inverted/reversed
     */
    public void setInverted(boolean inverted) {
        servo.setDirection(inverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    /**
     * @return whether the servo is inverted/reversed
     */
    public boolean getInverted() {
        return servo.getDirection().equals(Servo.Direction.REVERSE);
    }

    /**
     * @param pwmRange the PWM range the servo should be set to
     * @return this object for chaining purposes
     */
    public CachedServo setPwm(PwmControl.PwmRange pwmRange) {
        getController().setServoPwmRange(servo.getPortNumber(), pwmRange);
        return this;
    }


    /**
     * @return the extended servo controller object for the servo
     */
    public ServoControllerEx getController() {
        return (ServoControllerEx) servo.getController();
    }

    /**
     * @return the port the servo controller is controlling the servo from
     */
    public int getPortNumber() {
        return this.servo.getPortNumber();
    }

    /**
     * @param cachingTolerance the new caching tolerance between servo writes
     * @return this object for chaining purposes
     */
    public CachedServo setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    /**
     * @return the caching tolerance of the servo before it writes a new power to the CR servo
     */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    public void disable() {
        servo.close();
    }

    public String getDeviceType() {
        return "Extended Servo; " + id + " from " + servo.getPortNumber();
    }
    /**
     * @return the SDK, unwrapped Servo object
     */
    public Servo getServo() {
        return servo;
    }
}
