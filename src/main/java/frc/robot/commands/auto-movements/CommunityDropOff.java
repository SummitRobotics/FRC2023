import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.MoveArmHome;
import frc.robot.commands.drivetrain.FollowDynamicTrajectoryThreaded;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.FieldElementPositions;

public class CommunityDropOff extends SequentialCommandGroup {
    
    public CommunityDropOff(Drivetrain drivetrain, Arm arm) {

        final int value = Integer.parseInt(NetworkTableInstance.getDefault()
            .getTable("stationSelector").getEntry("id").getString("00"));

        // Encoding: first digit is x, second is y, 11 is top left from driver's perspective.
        // 00 means nothing is selected or we couldn't read the table data.
        final int xCoordIndex = value - value % 10;
        final int yCoordIndex = value % 10;

        final Translation3d node;
        final Pose2d drivePoint;

        if (xCoordIndex != 0 && yCoordIndex != 0) {
            // xCoordIndex is used for z value because height depends on x
            if (DriverStation.getAlliance() == Alliance.Blue) {
                node = new Translation3d(
                    FieldElementPositions.BLUE_X_VALUES[xCoordIndex - 1],
                    FieldElementPositions.BLUE_Y_VALUES[yCoordIndex - 1],
                    FieldElementPositions.BLUE_Z_VALUES[xCoordIndex - 1]
                );
                drivePoint = new Pose2d(
                    FieldElementPositions.BLUE_X_VALUES[xCoordIndex - 1] + 0.6,
                    FieldElementPositions.BLUE_Y_VALUES[yCoordIndex - 1],
                    new Rotation2d(-4, 0)
                );
            } else {
                node = new Translation3d(
                    FieldElementPositions.RED_X_VALUES[xCoordIndex - 1],
                    FieldElementPositions.RED_Y_VALUES[yCoordIndex - 1],
                    FieldElementPositions.RED_Z_VALUES[xCoordIndex - 1]
                );
                drivePoint = new Pose2d(
                    FieldElementPositions.RED_X_VALUES[xCoordIndex - 1] - 0.6,
                    FieldElementPositions.RED_Y_VALUES[yCoordIndex - 1],
                    new Rotation2d(4, 0)
                );
            }

            addCommands(
                new MoveArmHome(arm),
                new InstantCommand(() -> drivetrain.highGear()),
                new FollowDynamicTrajectoryThreaded(
                    drivetrain::getPose,
                    () -> drivePoint,
                    () -> new ArrayList<Translation2d>(),
                    drivetrain.generateTrajectoryConfigHighGear(),
                    drivetrain
                ),
                // TODO - adjust grabber angle and wrist rotations
                new MoveArm(
                    arm,
                    Positions.Pose3d.fromFieldSpace(
                        node,
                        new Pose3d( // drivetrain pose in 3d
                            drivetrain.getPose().getX(),
                            drivetrain.getPose().getY(),
                            0,
                            new Rotation3d(0, 0, drivetrain.getPose().getRotation().getRadians())
                        )
                    ),
                    0.0,
                    0.0
                ),
                new InstantCommand(() -> arm.unclamp()),
                new MoveArmHome(arm)
            );
        }
    }
}
