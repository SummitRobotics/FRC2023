package frc.robot.utilities.homing;

/**
 * A functional interface that subsystems with homeable motors will implement
 */
public interface HomeableSubsystem {
    public HomeableCANSparkMax[] getHomeables();
}
