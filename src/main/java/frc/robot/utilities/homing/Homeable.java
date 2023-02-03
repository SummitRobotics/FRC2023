package frc.robot.utilities.homing;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * interface for homing a component.
 */
public interface Homeable {
    public double getCurrent();

    public double getVelocity();

    public void setPower(double power);

    public void setPower();

    public void setHome(double position);

    public void setSoftLimits(double reverse, double forward);

    public void disableSoftLimits();

    public void enableSoftLimits();

    public Subsystem getSubsystemObject();
}
