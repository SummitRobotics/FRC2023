package frc.robot.commands.automovements;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.FollowDynamicTrajectoryThreaded;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.FieldElementPositions;

public class CommunityDropOff extends SequentialCommandGroup {
    
    private static final double 
        SLOPE = 0.5,
        GRABBER_ANGLE = 0;

    public CommunityDropOff(Drivetrain drivetrain, Arm arm) {


        final int value = Integer.parseInt(NetworkTableInstance.getDefault()
            .getTable("customDS").getEntry("location").getString("00"));

        // Encoding: first digit is x, second is y, 11 is top left from driver's perspective.
        // 00 means nothing is selected or we couldn't read the table data.
        final int xCoordIndex = value - value % 10;
        final int yCoordIndex = value % 10;

        final Positions.Pose3d node;
        final Pose2d drivePoint;

        if (xCoordIndex != 0 && yCoordIndex != 0) {
            // xCoordIndex is used for z value because height depends on x
            if (DriverStation.getAlliance() == Alliance.Blue) {
                node = Positions.Pose3d.fromFieldSpace(new Translation3d(
                    FieldElementPositions.BLUE_X_VALUES[xCoordIndex - 1],
                    FieldElementPositions.BLUE_Y_VALUES[yCoordIndex - 1],
                    FieldElementPositions.BLUE_Z_VALUES[xCoordIndex - 1]
                ));
                double x = Math.abs(drivetrain.getPose().getX() - FieldElementPositions.BLUE_X_VALUES[xCoordIndex - 1] + 0.6);
                double y = drivetrain.getPose().getY() - FieldElementPositions.BLUE_Y_VALUES[yCoordIndex - 1];
                double slope = y / x;
                Rotation2d endRot;
                if (slope < -SLOPE) {
                    endRot = Rotation2d.fromDegrees(90);
                } else if (slope > SLOPE) {
                    endRot = Rotation2d.fromDegrees(-90);
                } else {
                    endRot = Rotation2d.fromDegrees(180);
                }
                drivePoint = new Pose2d(
                    FieldElementPositions.BLUE_X_VALUES[xCoordIndex - 1] + 0.6,
                    FieldElementPositions.BLUE_Y_VALUES[yCoordIndex - 1],
                    endRot
                );
            } else {
                node = Positions.Pose3d.fromFieldSpace(new Translation3d(
                    FieldElementPositions.RED_X_VALUES[xCoordIndex - 1],
                    FieldElementPositions.RED_Y_VALUES[yCoordIndex - 1],
                    FieldElementPositions.RED_Z_VALUES[xCoordIndex - 1]
                ));
                double x = Math.abs(drivetrain.getPose().getX() - FieldElementPositions.RED_X_VALUES[xCoordIndex - 1] + 0.6);
                double y = drivetrain.getPose().getY() - FieldElementPositions.RED_Y_VALUES[yCoordIndex - 1];
                double slope = y / x;
                Rotation2d endRot;
                if (slope < -SLOPE) {
                    endRot = Rotation2d.fromDegrees(90);
                } else if (slope > SLOPE) {
                    endRot = Rotation2d.fromDegrees(-90);
                } else {
                    endRot = Rotation2d.fromDegrees(0);
                }
                drivePoint = new Pose2d(
                    FieldElementPositions.RED_X_VALUES[xCoordIndex - 1] + 0.6,
                    FieldElementPositions.RED_Y_VALUES[yCoordIndex - 1],
                    endRot
                );
            }

            addCommands(
                new MoveArmUnsafe(arm, ARM_POSITION.HOME).unless(() -> arm.isWithinRange(node, GRABBER_ANGLE)),
                new InstantCommand(() -> drivetrain.highGear()).unless(() -> arm.isWithinRange(node, GRABBER_ANGLE)),
                new FollowDynamicTrajectoryThreaded(
                    drivetrain::getPose,
                    () -> drivePoint,
                    () -> new ArrayList<Translation2d>(),
                    drivetrain.generateTrajectoryConfigHighGear(),
                    drivetrain
                ).unless(() -> arm.isWithinRange(node, GRABBER_ANGLE)),
                // TODO - adjust grabber angle and wrist rotations
                new MoveArm(
                    arm,
                    node,
                    GRABBER_ANGLE
                ),
                // new InstantCommand(() -> arm.unclamp()),
                new MoveArmUnsafe(arm, ARM_POSITION.HOME)
            );
        }
    }
}
