package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.utils.CachedServo;
import org.firstinspires.ftc.teamcode.utils.CachedMotor;

@Configurable
public class Shooter {

    private CachedMotor shooterCachedMotor1, shooterCachedMotor2, shooterEncoder1, shooterEncoder2;
    private Servo hood;

    private CachedServo gate;

    VoltageSensor voltageSensor;

    private PIDFController pidf;
    private SimpleMotorFeedforward feedforward;

    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;
    public static double diffTurret = -0.006;
    // --- Flywheel PIDF coefficients ---
    public static double kP = 0.005;

    public static double kS = 0.0613641; // Static feedforward
    public static double kV = 0.000375058; // Velocity feedforward

    public static boolean enablePIDF = true;

    // --- Turret bounds ---
    public static double turretUpperBound = 0.95;
    public static double turretLowerBound = 0.05;
    public static double shooterincrease = 100;


    // --- Hood bounds ---
    public static double hoodLowerBound = 0.48;
    public static double hoodUpperBound = 0.85;
    private double baseHoodPosition = hoodLowerBound;

    public static Pose RedGoalPose = new Pose(-70.25, 70.25);
    public static Pose BlueGoalPose
            = new Pose(-70.25, -70.25);

    public enum Goal{
        RED (RedGoalPose),
        BLUE (BlueGoalPose);

        Pose position;
        Goal(Pose pose) {
            position = pose;
        }
    }

    public static double powerOffset = 0;
    public static double turretOffset = 0;
    public static double hoodOffset = 0;
    public static double limelightOffset = 0;


    public static double upperGateOpenPos = 0.68;
    public static double upperGateClosedPos = 0.56;


    private double prevTargetVelocity = 0.0;
    private long prevTargetTime;

    public boolean canReachPos = true;

    public Shooter(HardwareMap hardwareMap) {
        shooterCachedMotor1 = new CachedMotor(hardwareMap, "shooterMotor1");
        shooterCachedMotor2 = new CachedMotor(hardwareMap, "shooterMotor2");

        shooterEncoder1 = new CachedMotor(hardwareMap, "frontright");
        shooterEncoder2 = new CachedMotor(hardwareMap, "shooterMotor2");


        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        gate = new CachedServo(hardwareMap, "Gate");

        hood = hardwareMap.servo.get("hood");

        pidf = new PIDFController(kP, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        prevTargetTime = System.nanoTime();
    }

    public void setGate(boolean open){
        if (open){
            gate.setPosition(upperGateOpenPos);
        } else {
            gate.setPosition(upperGateClosedPos);
        }
    }

    public double[] getAngleDistance(Pose currPosition, Goal target){
        return getAngleDistance(currPosition, target.position);
    }

    public double[] getAngleDistance(Pose currPosition, Pose target){
        double dx = target.x()-currPosition.x();
        double dy = target.y()-currPosition.y();
        double angle = Math.atan2(dy, dx);
        double turretAngle = Math.toDegrees(-angle + currPosition.heading());

        while (Math.abs(turretAngle)>=180){
            if (turretAngle>0){
                turretAngle -= 360;
            }else{
                turretAngle +=360;
            }
        }

        double distance = Math.hypot(dx, dy);

        return new double[]{turretAngle, distance};
    }


    public void aimAtTarget(Pose currPosition, Goal target){
        aimAtTarget(currPosition, target.position);
    }

    public void aimAtTarget(Pose currPosition, Pose target){
        double[] angleDistance = getAngleDistance(currPosition, target);
        double angle = angleDistance[0];
        double distance = angleDistance[1];
        setTargetVelocity(Tables.getShooterVelocity(distance) + powerOffset);
        setHood(Tables.getHoodPosition(distance) + hoodOffset);
    }

    public void setTargetVelocity(double target) {
        targetVelocity = target;
        pidf.reset();
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public void setDirectPower(double power) {
        power = power * 12/voltageSensor.getVoltage();
        shooterCachedMotor1.set(power);
        shooterCachedMotor2.set(-power);
    }

    public void setHood(double pos){
        System.out.println("hood"+pos);
        hood.setPosition(Range.clip(pos, hoodLowerBound, hoodUpperBound));
    }

    public void update() {
        // Measure velocity
        currentVelocity = getCurrentVelo();

        long currTime = System.nanoTime();
        double dt = (currTime - prevTargetTime) / 1e9;
        double accel = 0.0;
        if (prevTargetTime != 0 && dt > 0) {
            accel = (targetVelocity - prevTargetVelocity) / dt;
        }
        prevTargetVelocity = targetVelocity;
        prevTargetTime = currTime;

        double outputPower;

        if (targetVelocity <= 0) {
            outputPower = 0;
        } else {
            outputPower = feedforward.calculate(targetVelocity, accel);
            if (enablePIDF){
                double error = (targetVelocity - currentVelocity);

                if (Math.abs(error) > 60)
                    outputPower = Math.signum(error);
                else
                    outputPower += pidf.calculate(currentVelocity, targetVelocity);
            }
        }

        setDirectPower(Math.max(outputPower,0));
        gate.update();
        update_motors();
    }

    public void update_motors(){
        shooterCachedMotor1.update();
        shooterCachedMotor2.update();
    }

    public double getTargetVelo() {
        return targetVelocity;
    }

    public double getCurrentVelo() {
        System.out.println("1: "+shooterEncoder1.getVelocity());
        System.out.println("2: "+shooterEncoder2.getVelocity());

        if(shooterEncoder1.getVelocity()<10){
            return Math.abs(shooterEncoder2.getVelocity());
        }
        else{
            return Math.abs(shooterEncoder1.getVelocity());
        }
    }
    @Configurable
    public static class Tables {
        private static final double[] DISTANCES = {
                123.3, 127.1, 131.0, 134.2, 137.1,
                141.1, 145.4, 149.3, 153.2, 158.3
        };

        private static final double[] HOOD = {
                0.68, 0.66, 0.66, 0.66, 0.66,
                0.66, 0.66, 0.63, 0.65, 0.75
        };

        private static final double[] VELOCITY = {
                1770, 1790, 1790, 1810, 1830,
                1870, 1920, 1970, 2030, 2160
        };

        public static double getHoodPosition(double distance) {
            return interpolate(distance, DISTANCES, HOOD);
        }


        public static double getShooterVelocity(double distance) {
            return interpolate(distance, DISTANCES, VELOCITY);
        }
        private static double interpolate(double x, double[] xValues, double[] yValues) {

            // Clamp below smallest value
            if (x <= xValues[0])
                return yValues[0];

            // Clamp above largest value
            if (x >= xValues[xValues.length - 1])
                return yValues[yValues.length - 1];

            // Find interval
            for (int i = 0; i < xValues.length - 1; i++) {

                if (x >= xValues[i] && x <= xValues[i + 1]) {

                    double x1 = xValues[i];
                    double x2 = xValues[i + 1];

                    double y1 = yValues[i];
                    double y2 = yValues[i + 1];

                    double t = (x - x1) / (x2 - x1);

                    return y1 + t * (y2 - y1);
                }
            }

            // Should never happen
            return yValues[yValues.length - 1];
        }
    }
}