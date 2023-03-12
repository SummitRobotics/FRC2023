package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.FieldElementPositions;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.FollowPathPlannerTrajectory;
import frc.robot.subsystems.Drivetrain;

public class PlaceNMoveNGrabNPlace extends SequentialCommandGroup {

    public enum Type {
        FarFromSubstation,
        CloseToSubstation
    }

    public PlaceNMoveNGrabNPlace(Arm arm, Drivetrain drivetrain, Type type) {

        PathPlannerTrajectory firstTraj;

        PathPlannerTrajectory secondTraj;
        
        ARM_POSITION placePos;
        Positions.Pose3d grabPos;

        if (type == Type.FarFromSubstation) {
            firstTraj = PathPlanner.loadPath("FarFromSubstation", new PathConstraints(4, 3));
            secondTraj = PathPlanner.loadPath("FarFromSubstationBack", new PathConstraints(4, 3));
            if (DriverStation.getAlliance() == Alliance.Blue) {
                placePos = ARM_POSITION.LEFT_HIGH;
                grabPos = Positions.Pose3d.fromFieldSpace(FieldElementPositions.BLUE_FAR_FROM_SUBSTATION);
            } else {
                placePos = ARM_POSITION.RIGHT_HIGH;
                grabPos = Positions.Pose3d.fromFieldSpace(FieldElementPositions.RED_FAR_FROM_SUBSTATION);
            }
        } else {
            firstTraj = PathPlanner.loadPath("CloseToSubstation", new PathConstraints(4, 3));
            secondTraj = PathPlanner.loadPath("CloseToSubstationBack", new PathConstraints(4, 3));
            if (DriverStation.getAlliance() == Alliance.Blue) {
                placePos = ARM_POSITION.RIGHT_HIGH;
                grabPos = Positions.Pose3d.fromFieldSpace(FieldElementPositions.BLUE_CLOSE_TO_SUBSTATION);
            } else {
                placePos = ARM_POSITION.LEFT_HIGH;
                grabPos = Positions.Pose3d.fromFieldSpace(FieldElementPositions.RED_CLOSE_TO_SUBSTATION);
            }
        }

        addCommands(
            new InstantCommand(drivetrain::highGear),
            new ArmOutOfStart(arm),
            // TODO - figure out if we need to move forward to align with the cone pole
            new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
            new MoveArmUnsafe(arm, placePos),
            new InstantCommand(arm::unclamp),
            new ParallelCommandGroup(
                new MoveArmUnsafe(arm, ARM_POSITION.HOME),
                new FollowPathPlannerTrajectory(drivetrain, firstTraj, true)
            ),
            new MoveArmUnsafe(arm, ArmConfiguration.fromEndPosition(grabPos, 0)),
            new InstantCommand(arm::clamp),
            new ParallelCommandGroup(
                new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
                new FollowPathPlannerTrajectory(drivetrain, secondTraj, true)
            ),
            new InstantCommand(arm::unclamp)
        );
    }
}
