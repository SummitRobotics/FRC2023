package frc.robot.commands.automovements;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.FollowDynamicTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.FieldElementPositions;

public class SubstationPickup extends SequentialCommandGroup {

    public enum Side {
        Left,
        Right
    }

    public SubstationPickup(Drivetrain drivetrain, Arm arm, Side side) {

        final Translation3d substation;
        final Pose2d drivePoint;
        final Pose2d preDrivePoint;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            if (side == Side.Left) {
                substation = FieldElementPositions.BLUE_LEFT_SUBSTATION;
                drivePoint = FieldElementPositions.BLUE_LEFT_DRIVE_POINT;
                preDrivePoint = FieldElementPositions.BLUE_LEFT_DRIVE_POINT_PRE;
            } else {
                substation = FieldElementPositions.BLUE_RIGHT_SUBSTATION;
                drivePoint = FieldElementPositions.BLUE_RIGHT_DRIVE_POINT;
                preDrivePoint = FieldElementPositions.BLUE_RIGHT_DRIVE_POINT_PRE;
            }
        } else {
            if (side == Side.Left) {
                substation = FieldElementPositions.RED_LEFT_SUBSTATION;
                drivePoint = FieldElementPositions.RED_LEFT_DRIVE_POINT;
                preDrivePoint = FieldElementPositions.RED_LEFT_DRIVE_POINT_PRE;
            } else {
                substation = FieldElementPositions.RED_RIGHT_SUBSTATION;
                drivePoint = FieldElementPositions.RED_RIGHT_DRIVE_POINT;
                preDrivePoint = FieldElementPositions.RED_RIGHT_DRIVE_POINT_PRE;
            }
        }

        addCommands(
            new MoveArmUnsafe(arm, ARM_POSITION.HOME),
            new InstantCommand(() -> drivetrain.highGear()),
            new FollowDynamicTrajectory(
                drivetrain::getPose,
                () -> preDrivePoint,
                () -> new ArrayList<Translation2d>(),
                drivetrain.generateTrajectoryConfigHighGear(),
                drivetrain
            ),
            new MoveArmUnsafe(arm, FieldElementPositions.substationPregrab),
            new InstantCommand(() -> arm.unclamp()),
            new FollowDynamicTrajectory(
                drivetrain::getPose,
                () -> drivePoint,
                () -> new ArrayList<Translation2d>(),
                drivetrain.generateTrajectoryConfigHighGear(),
                drivetrain
            ),
            // TODO - adjust grabber angle and wrist rotations
            new MoveArmUnsafe(
                arm,
                Positions.Pose3d.fromFieldSpace(substation),
                0.0
            ),
            new InstantCommand(() -> arm.clamp())
            );
    }
}
