package frc.robot.commands.drivetrain;

import java.util.function.BiConsumer;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to have the drivetrain follow a PathPlannerLib trajectory.
 * This was based on the example code found at
 * https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage.
 */
public class FollowPathPlannerTrajectory extends SequentialCommandGroup {
    public FollowPathPlannerTrajectory(Drivetrain drivetrain, PathPlannerTrajectory traj, boolean resetPose, boolean allianceMirror) {
        addCommands(
            // Pose should be reset to the start point if this is the first trajectory to run
            new InstantCommand(() -> {
                if (resetPose) {
                    if (allianceMirror) {
                        drivetrain.setPose(PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance()).getInitialPose());
                    } else {
                        drivetrain.setPose(traj.getInitialPose());
                    }
                }
            }),
            new PPRamseteCommand(
                traj, 
                drivetrain::getPose, // Pose supplier
                new RamseteController(),
                Drivetrain.HighFeedForward,
                Drivetrain.DriveKinematics, // DifferentialDriveKinematics
                drivetrain::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                // TODO - tune PID controllers for trajectory following
                new PIDController(Drivetrain.HIGH_P, Drivetrain.HIGH_I, Drivetrain.HIGH_D),
                new PIDController(Drivetrain.HIGH_P, Drivetrain.HIGH_I, Drivetrain.HIGH_D),
                new BiConsumer<Double, Double>() { // Voltage biconsumer
                    @Override
                    public void accept(Double left, Double right) {
                        drivetrain.setMotorVolts(left, right);
                    }
                },
                true, // Should the path be automatically mirrored depending on alliance color? Optional, defaults to true
                drivetrain // Requires the drive subsystem
            )
        );
    }
}
