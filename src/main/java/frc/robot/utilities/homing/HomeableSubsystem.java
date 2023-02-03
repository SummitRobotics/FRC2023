package frc.robot.utilities.homing;

/**
 * A functional interface that subsystems with homeable components will implement
 */
public interface HomeableSubsystem {
    public Homeable[] getHomeables();
}
