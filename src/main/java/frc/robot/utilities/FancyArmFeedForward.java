package frc.robot.utilities;

public class FancyArmFeedForward {
    public final double kS;
    public final double kV;
    public final double kA;
    public final double kG;
    public final double weight; // In Newtons
    public final double torque; // In Newton meters

    public FancyArmFeedForward(double kS, double kV, double kA, double kG, double weight, double torque) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.weight = weight;
        this.torque = torque;
    }

    public FancyArmFeedForward(double kS, double kV, double kA, double weight, double torque){
        this(kS, kV, kA, 1, weight, torque);
    }

    public double calculate(double position, double velocityRotPerSec, double accelRotPerSecSquared, double CGDistance, double CGAngle, double gearRatio) {

        double kG = this.kG * ((CGDistance * weight * 12 * Math.cos(CGAngle)) / (torque * gearRatio));

        return kS * Math.signum(velocityRotPerSec) + kV * velocityRotPerSec + kA * accelRotPerSecSquared + kG;
    }

    public double calculate(double position, double velocityRotPerSec, double CGDistance, double CGAngle, double gearRatio) {
        return calculate(position, velocityRotPerSec, 0, CGDistance, CGAngle, gearRatio);
    }

    public double calculate(double position, double CGDistance, double CGAngle, double gearRatio) {
        return calculate(position, 0, 0, CGDistance, CGAngle, gearRatio);
    }

    public double calculate(double position, double velocityRotPerSec, FFData data) {
        return calculate(position, velocityRotPerSec, data.distance, data.angle, data.gearRatio);
    }

    public double calculate(double position, FFData data) {
        return calculate(position, 0, data);
    }

    public static class FFData {
        public final double distance;
        public final double angle;
        public final double gearRatio;

        public FFData(double distance, double angle, double gearRatio) {
            this.distance = distance;
            this.angle = angle;
            this.gearRatio = gearRatio;
        }
    }
}
