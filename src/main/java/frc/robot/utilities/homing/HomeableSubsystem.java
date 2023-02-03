package frc.robot.utilities.homing;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An interface that subsystems with homeable motors will implement
 */
public interface HomeableSubsystem {
    public HomeableCANSparkMax[] getHomeables();

    public Subsystem getSubsystemObject();
}
