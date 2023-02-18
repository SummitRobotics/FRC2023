import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.MoveArmHome;
import frc.robot.commands.drivetrain.FollowDynamicTrajectoryThreaded;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.FieldElementPositions;

public class SubstationPickup extends SequentialCommandGroup {

    public enum Side {
        Left,
        Right
    }

    public SubstationPickup(Drivetrain drivetrain, Arm arm, Side side) {

        Pose2d drivePoint;
        Translation3d gamePiece;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            if (side == Side.Left) {
                drivePoint = FieldElementPositions.BLUE_LEFT_DRIVE_POINT;
                gamePiece = FieldElementPositions.BLUE_LEFT_GAME_PIECE;
            } else {
                drivePoint = FieldElementPositions.BLUE_RIGHT_DRIVE_POINT;
                gamePiece = FieldElementPositions.BLUE_RIGHT_GAME_PIECE;
            }
        } else {
            if (side == Side.Left) {
                drivePoint = FieldElementPositions.RED_LEFT_DRIVE_POINT;
                gamePiece = FieldElementPositions.RED_LEFT_GAME_PIECE;
            } else {
                drivePoint = FieldElementPositions.RED_RIGHT_DRIVE_POINT;
                gamePiece = FieldElementPositions.RED_RIGHT_GAME_PIECE;
            }
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
                    gamePiece
                ),
                0.0,
                0.0
            ),
            new InstantCommand(() -> arm.clamp()),
            new MoveArmHome(arm)
        );
    }
}