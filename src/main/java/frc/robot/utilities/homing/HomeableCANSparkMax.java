package frc.robot.utilities.homing;

import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.RollingAverage;

/**
 * A class containing home by current settings for a CANSparkMax
 */
public class HomeableCANSparkMax {

    public enum Type {
        ByCurrent,
        BySwitch,
    }

    private final RollingAverage currentAverage = new RollingAverage(10, false);
    private Type type;
    private CANSparkMax motor;
    private Subsystem subsystem;
    private double homingPower;
    private double currentThreshold;
    private double forwardLimit;
    private double reverseLimit;
    private BooleanSupplier switchCondition;
    private boolean softLimits;
    private boolean isFinished;

    // by current, no soft limits
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        double currentThreshold
    ) {

        type = Type.ByCurrent;
        softLimits = false;
        this.motor = motor;
        this.subsystem = subsystem;
        this.homingPower = homingPower;
        this.currentThreshold = currentThreshold;
        forwardLimit = 0;
        reverseLimit = 0;
        switchCondition = () -> false;
        isFinished = false;
    }

    // by switch, no soft limits
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        BooleanSupplier switchCondition
    ) {
        type = Type.BySwitch;
        softLimits = false;
        this.motor = motor;
        this.subsystem = subsystem;
        this.homingPower = homingPower;
        currentThreshold = 0;
        forwardLimit = 0;
        reverseLimit = 0;
        this.switchCondition = switchCondition;
        isFinished = false;
    }

    // by current, soft limits
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        double currentThreshold,
        double forwardLimit,
        double reverseLimit
    ) {
        type = Type.ByCurrent;
        softLimits = true;
        this.motor = motor;
        this.subsystem = subsystem;
        this.homingPower = homingPower;
        this.currentThreshold = currentThreshold;
        this.forwardLimit = forwardLimit;
        this.reverseLimit = reverseLimit;
        switchCondition = () -> false;
        isFinished = false;
    }

    // by switch, soft limits
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        BooleanSupplier switchCondition,
        double forwardLimit,
        double reverseLimit
    ) {
        type = Type.BySwitch;
        softLimits = true;
        this.motor = motor;
        this.subsystem = subsystem;
        this.homingPower = homingPower;
        currentThreshold = 0;
        this.forwardLimit = forwardLimit;
        this.reverseLimit = reverseLimit;
        this.switchCondition = switchCondition;
        isFinished = false;
    }

    public boolean getIsFinished() {
        return isFinished;
    }

    public void setIsFinished() {
        this.isFinished = true; 
    }

    public Type getType() {
        return type;
    }

    public boolean hasSoftLimits() {
        return softLimits;
    }

    public void updateCurrent() {
        currentAverage.update(motor.getOutputCurrent());
    }

    public void resetCurrent() {
        currentAverage.reset();
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void setPower() {
        motor.set(homingPower);
    }

    public void zeroHome() {
        motor.getEncoder().setPosition(0);
    }

    public void disableSoftLimits() {
        motor.enableSoftLimit(SoftLimitDirection.kForward, false);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public void enableSoftLimits() {
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) forwardLimit);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverseLimit);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public boolean stopCondition() {
        switch (type) {
            case ByCurrent:
                return currentAverage.getAverage() >= currentThreshold;
            case BySwitch:
                return switchCondition.getAsBoolean();
            default:
                return false;
        }
    }

    public Subsystem getSubsystemObject() {
        return subsystem;
    }
}
