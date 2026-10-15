package org.firstinspires.ftc.teamcode.utils;

//import static com.pedropathing.math.MathFunctions.clamp;

import androidx.annotation.NonNull;

//import com.pedropathing.utils.Utils;
//import com.pedropathing.math.MathFunctions;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.Supplier;

public class CachedMotor {

    // Instance fields
    public DcMotorEx motor;
    public Encoder encoder;
    private double cachingTolerance = 0.01; // The minimum difference between the current and requested motor power between motor writes
    private double prevPower = 0;
    private double currPower = 0;


    // Constructors
    public CachedMotor() {}

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public CachedMotor(@NonNull HardwareMap hMap, String id) {
        motor = (DcMotorEx) hMap.get(DcMotor.class, id);
        encoder = new Encoder(motor::getCurrentPosition);
    }

    // Public methods

    /**
     * Resets the external encoder wrapper value.
     */
    public void resetEncoder() {
        encoder.reset();
    }

    /**
     * Resets the internal position of the motor.
     */
    public void stopAndResetEncoder() {
        encoder.resetVal = 0;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * A wrapper method for the zero power behavior
     *
     * @param behavior the behavior desired
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior.getBehavior());
    }

    /**
     * @return the current position of the motor in ticks
     */
    public int getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * @return the corrected velocity for overflow
     */
    public double getCorrectedVelocity() {
        return encoder.getCorrectedVelocity();
    }

    /**
     * Common method for getting the current set speed of a motor.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double get() {
        return currPower;
    }

    /**
     * Common method for inverting direction of a motor.
     *
     * @param isInverted The state of inversion true is inverted.
     */
    public void setInverted(boolean isInverted) {
        motor.setDirection(isInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    /**
     * Common method for returning if a motor is in the inverted state or not.
     *
     * @return isInverted The state of the inversion true is inverted.
     */
    public boolean getInverted() {
        return DcMotor.Direction.REVERSE == motor.getDirection();
    }

    /**
     * Disable the motor.
     */
    public void disable() {
        motor.close();
    }


    public String getDeviceType() {
        return "Motor " + motor.getDeviceName() + " from " + motor.getManufacturer()
                + " in port " + motor.getPortNumber();
    }

    /**
     * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
     * motor.
     */
    public void stopMotor() {
        set(0);
    }

    /**
     * Sets the power of the motor.
     *
     * @param output the power to set the motor to, between -1.0 and 1.0
     */
    public void set(double output) {
        currPower= Utils.clamp(output, -1.0, 1.0);
    }

    /**
     * Updates the motor.
     */
    public void update() {
        if (currPower == prevPower) {
            return;
        }
        if (currPower==0){
            motor.setPower(0);
            prevPower = 0;
            return;
        }
        if (Math.abs(currPower - prevPower) > cachingTolerance) {
            motor.setPower(currPower);
            prevPower = currPower;
        }
    }

    /**
     * @return the velocity of the motor in ticks per second
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * @return the caching tolerance of the motor before it writes a new power to the motor
     */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    /**
     * @param cachingTolerance the new caching tolerance between motor writes
     * @return this object for chaining purposes
     */
    public CachedMotor setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    // Inner classes and enums

    public enum Direction {
        FORWARD(1), REVERSE(-1);

        private int val;

        Direction(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public enum ZeroPowerBehavior {
        UNKNOWN(DcMotor.ZeroPowerBehavior.UNKNOWN),
        BRAKE(DcMotor.ZeroPowerBehavior.BRAKE),
        FLOAT(DcMotor.ZeroPowerBehavior.FLOAT);

        private final DcMotor.ZeroPowerBehavior m_behavior;

        ZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
            m_behavior = behavior;
        }

        public DcMotor.ZeroPowerBehavior getBehavior() {
            return m_behavior;
        }
    }

    public class Encoder {

        private Supplier<Integer> m_position;
        private int resetVal, lastPosition;
        private Direction direction;
        private double lastTimeStamp, veloEstimate, dpp, accel, lastVelo;

        /**
         * The encoder object for the motor.
         *
         * @param position the position supplier which just points to the
         *                 current position of the motor in ticks
         */
        public Encoder(Supplier<Integer> position) {
            m_position = position;
            resetVal = 0;
            lastPosition = 0;
            veloEstimate = 0;
            direction = Direction.FORWARD;
            lastTimeStamp = (double) System.nanoTime() / 1E9;
        }

        /**
         * @return the current position of the encoder
         */
        public int getPosition() {
            int currentPosition = m_position.get();
            if (currentPosition != lastPosition) {
                double currentTime = (double) System.nanoTime() / 1E9;
                double dt = currentTime - lastTimeStamp;
                veloEstimate = (currentPosition - lastPosition) / dt;
                lastPosition = currentPosition;
                lastTimeStamp = currentTime;
            }
            return direction.getMultiplier() * currentPosition - resetVal;
        }

        /**
         * Resets the encoder without having to stop the motor.
         */
        public void reset() {
            resetVal += getPosition();
        }

        /**
         * Sets the direction of the encoder to forward or reverse
         *
         * @param direction the desired direction
         */
        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        /**
         * @return the raw velocity of the motor reported by the encoder
         */
        public double getRawVelocity() {
            double velo = getVelocity();
            if (velo != lastVelo) {
                double currentTime = (double) System.nanoTime() / 1E9;
                double dt = currentTime - lastTimeStamp;
                accel = (velo - lastVelo) / dt;
                lastVelo = velo;
                lastTimeStamp = currentTime;
            }
            return velo;
        }

        /**
         * @return the estimated acceleration of the motor in ticks per second squared
         */
        public double getAcceleration() {
            return accel;
        }

        private final static int CPS_STEP = 0x10000;

        /**
         * Corrects for velocity overflow
         *
         * @return the corrected velocity
         */
        public double getCorrectedVelocity() {
            double real = getRawVelocity();
            while (Math.abs(veloEstimate - real) > CPS_STEP / 2.0) {
                real += Math.signum(veloEstimate - real) * CPS_STEP;
            }
            return real;
        }
    }
    public double getCurrentDraw(){
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public void setCurrentAlert(double current){
        motor.setCurrentAlert(current, CurrentUnit.AMPS);
    }

    public boolean isOverCurrent(){
        return motor.isOverCurrent();
    }
}