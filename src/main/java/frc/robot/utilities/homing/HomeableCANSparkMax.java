package frc.robot.utilities.homing;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A Homeable class for CANSparkMax
 */
public class HomeableCANSparkMax implements Homeable {

    private CANSparkMax motor;
    private Subsystem subsystem;
    private double homingPower;

    public HomeableCANSparkMax(CANSparkMax motor, Subsystem subsystem, double homingPower) {
        this.motor = motor;
        this.subsystem = subsystem;
    }

    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    // Power defaults to provided homing power

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public void setPower() {
        motor.set(homingPower);
    }

    @Override
    public void setHome(double position) {
        motor.getEncoder().setPosition(position);
    }

    @Override
    public void setSoftLimits(double forward, double reverse) {
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) forward);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverse);
    }

    @Override
    public void disableSoftLimits() {
        motor.enableSoftLimit(SoftLimitDirection.kForward, false);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    @Override
    public void enableSoftLimits() {
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    @Override
    public Subsystem getSubsystemObject() {
        return subsystem;
    }
    
}
