package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.CommandThreader;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to follow a trajectory.
 * This command is threaded using Command Threader
 * This means the trajectory calculations runs every ~5ms
 */
public class FollowTrajectoryThreaded extends CommandBase {

    private final Trajectory trajectory;
    private final Drivetrain drivetrain;
    private final int period;
    private CommandThreader commandThreader;

    /**
     * command to follow a trajectory object.
     * The trajectory has been saved to the roborio with threading to make
     * it more precise
     *
     * @param drivetrain drivetrain to control
     * @param trajectory path to the saved SerializableMultiGearTrajectory object
     */
    public FollowTrajectoryThreaded(Drivetrain drivetrain, Trajectory trajectory) {
        super();

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

        this.period = 5;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        drivetrain.getFieldWidget().getObject("trajectory").setTrajectory(trajectory);

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        trajectory,
                        drivetrain::getPose,
                        // TODO tune controller values
                        new RamseteController(),
                        Drivetrain.DriveKinematics,
                        drivetrain::setMotorTargetSpeed,
                        drivetrain);

        drivetrain.setPose(trajectory.getInitialPose());

        // Wraps the command, so we can update odometry every cycle.
        Runnable onExecute =
                () -> {
                    drivetrain.updateOdometry();
                    ramseteCommand.execute();
                };

        // Wraps the ramseteCommand in a functional command,
        // so we can update drivetrain odometry still.
        FunctionalCommand command = new FunctionalCommand(
                ramseteCommand::initialize,
                onExecute,
                ramseteCommand::end,
                ramseteCommand::isFinished,
                drivetrain);

        // Creates the command threader
        commandThreader = new CommandThreader(command, period, 10);
        commandThreader.initialize();

        System.out.println("thread started");
    }

    @Override
    public boolean isFinished() {
        // checks if thread is running or ended
        return commandThreader.isFinished();
    }

    @Override
    public void end(boolean interrupted) {

        // stops the thread
        commandThreader.end(interrupted);

        // stops the drivetrain motors
        drivetrain.stop();
    }
}
