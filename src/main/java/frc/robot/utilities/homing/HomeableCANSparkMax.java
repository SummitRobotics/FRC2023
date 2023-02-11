package frc.robot.utilities.homing;

import java.util.Comparator;
import java.util.function.BooleanSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.RollingAverage;

/**
 * A class containing homing settings for a CANSparkMax.
 * Can be by current or switch, with or without soft limits, and with or without an order.
 * The order must be positive and is evaluated from lowest to highest.
 */
public class HomeableCANSparkMax {

    public enum Type {
        ByCurrent,
        BySwitch,
    }

    public static Comparator<HomeableCANSparkMax> ORDER_COMPARATOR = new Comparator<>() {
        @Override
        public int compare(HomeableCANSparkMax left, HomeableCANSparkMax right) {
            return left.order - right.order;
        }
    };

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
    private int order;

    // by current, no soft limits, no order
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
        order = 0;
    }

    // by switch, no soft limits, no order
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
        order = 0;
    }

    // by current, soft limits, no order 
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        double currentThreshold,
        double forwardLimit,
        double reverseLimit
    ) {
        this(motor, subsystem, homingPower, currentThreshold);
        this.forwardLimit = forwardLimit;
        this.reverseLimit = reverseLimit;
        softLimits = true;
    }

    // by switch, soft limits, no order
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        BooleanSupplier switchCondition,
        double forwardLimit,
        double reverseLimit
    ) {
        this(motor, subsystem, homingPower, switchCondition);
        this.forwardLimit = forwardLimit;
        this.reverseLimit = reverseLimit;
        softLimits = true;
    }

    // by current, no soft limits, order
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        double currentThreshold,
        int order
    ) {
        this(motor, subsystem, homingPower, currentThreshold);
        this.order = order;
    }

    // by switch, no soft limits, order
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        BooleanSupplier switchCondition,
        int order
    ) {
        this(motor, subsystem, homingPower, switchCondition);
        this.order = order;
    }

    // by current, soft limits, order
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        double currentThreshold,
        double forwardLimit,
        double reverseLimit,
        int order
    ) {
        this(motor, subsystem, homingPower, currentThreshold, forwardLimit, reverseLimit);
        this.order = order;
    }

    // by switch, soft limits, order
    public HomeableCANSparkMax(
        CANSparkMax motor,
        Subsystem subsystem,
        double homingPower,
        BooleanSupplier switchCondition,
        double forwardLimit,
        double reverseLimit,
        int order
    ) {
        this(motor, subsystem, homingPower, switchCondition, forwardLimit, reverseLimit);
        this.order = order;
    }

    public void init() {
        if (type == Type.ByCurrent) currentAverage.reset();
        if (softLimits) {
            motor.enableSoftLimit(SoftLimitDirection.kForward, false);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        }
    }

    public void end() {
        motor.set(0);
        if (softLimits) {
            motor.setSoftLimit(SoftLimitDirection.kForward, (float) forwardLimit);
            motor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverseLimit);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        }
        motor.getEncoder().setPosition(0);
    }

    public int getOrder() {
        return order;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public void updateStopCondition() {
        if (
            type == Type.ByCurrent ? currentAverage.getAverage() >= currentThreshold
            : switchCondition.getAsBoolean()
        ) {
            isFinished = true;
        }
    }

    public Type getType() {
        return type;
    }

    public void updateCurrent() {
        currentAverage.update(motor.getOutputCurrent());
    }

    public void setPower(double value) {
        motor.set(value);
    }
    public void setPower() {
        motor.set(homingPower);
    }

    public Subsystem getSubsystemObject() {
        return subsystem;
    }
}
