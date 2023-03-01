package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.Colors;
import java.util.List;
import java.util.function.Supplier;

/**
 * Command to follow a trajectory.
 * This command is threaded using Command Threader
 * This means the trajectory calculations runs every ~5ms
 */
public class FollowDynamicTrajectory extends CommandBase {

    private final Drivetrain drivetrain;
    private Command command;

    private final Supplier<Trajectory> generateTrajectory;

    /**
     * Command to follow a trajectory object with a dynamic starting, midpoint and end pos.
     * The trajectory has been saved to the roborio with threading to make
     * it more precise
     *
     * @param drivetrain drivetrain to control
     * @param startingPos A supplier for the starting pos of the robot
     * @param midpoints A supplier for the midpoints for the trajectory
     * @param endingPos A supplier for the ending pos of the robot
     * @param config The trajectory config
     */
    public FollowDynamicTrajectory(
        Supplier<Pose2d> startingPos,
        Supplier<Pose2d> endingPos,
        Supplier<List<Translation2d>> midpoints,
        TrajectoryConfig config,
        Drivetrain drivetrain
    ) {
        this.drivetrain = drivetrain;

        generateTrajectory = () ->
            TrajectoryGenerator.generateTrajectory(startingPos.get(), midpoints.get(), endingPos.get(), config);

        addRequirements(drivetrain);
    }

    /**
     * Command to follow a trajectory object with dynamic waypoints.
     * The trajectory has been saved to the roborio with threading to make
     * it more precise.
     *
     * @param waypoints A supplier of the waypoints for the trajectory
     * @param config The trajectory config
     * @param drivetrain The drivetrain subsystem
     */

    public FollowDynamicTrajectory(
        Supplier<List<Pose2d>> waypoints,
        TrajectoryConfig config,
        Drivetrain drivetrain
    ) {
        this.drivetrain = drivetrain;

        generateTrajectory = () -> TrajectoryGenerator.generateTrajectory(waypoints.get(), config);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Trajectory trajectory = generateTrajectory.get();

        // System.out.println(trajectory.getStates());

        drivetrain.getFieldWidget().getObject("trajectory").setTrajectory(trajectory);

        RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                // TODO tune controller values
                new RamseteController(),
                Drivetrain.DriveKinimatics,
                drivetrain::setMotorTargetSpeed,
                drivetrain);

        //drivetrain.setPose(trajectory.getInitialPose());

        // Wraps the command, so we can update odometry every cycle.
        Runnable onExecute =
            () -> {
                ramseteCommand.execute();
            };

        // Wraps the ramseteCommand in a functional command,
        // so we can update drivetrain odometry still.
        command = new FunctionalCommand(
            ramseteCommand::initialize,
            onExecute,
            ramseteCommand::end,
            ramseteCommand::isFinished,
            drivetrain);

        // Creates the command threader
        command.initialize();
    }

    @Override
    public boolean isFinished() {
        if (command == null) {
            return true;
        }
        return command.isFinished();
    }

    @Override
    public void execute() {
        if (command != null) {
            command.execute();
        }
        // drivetrain.setMotorTargetSpeed(0.5, 0.5);
    }

    @Override
    public void end(boolean interrupted) {


        if (command != null) {
            command.end(interrupted);
        }

        // stops the drivetrain motors
        drivetrain.stop();
    }
}
