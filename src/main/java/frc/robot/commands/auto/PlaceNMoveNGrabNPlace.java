package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.FollowPathPlannerTrajectory;
import frc.robot.subsystems.Drivetrain;

public class PlaceNMoveNGrabNPlace extends SequentialCommandGroup {

    public enum Type {
        FarFromSubstation,
        CloseToSubstation
    }

    public PlaceNMoveNGrabNPlace(Arm arm, Drivetrain drivetrain, Type type, Alliance alliance) {

        PathPlannerTrajectory firstTraj;
        PathPlannerTrajectory secondTraj;
        ARM_POSITION placePos;
        ARM_POSITION grabPos;

        if (type == Type.FarFromSubstation) {
            firstTraj = PathPlanner.loadPath("FarFromSubstation", new PathConstraints(3, 3));
            secondTraj = PathPlanner.loadPath("FarFromSubstationBack", new PathConstraints(3, 3));
            if (alliance == Alliance.Blue) {
                placePos = ARM_POSITION.AUTO_PLACE_LEFT;
                grabPos = ARM_POSITION.AUTO_GRAB_LEFT;
            } else {
                placePos = ARM_POSITION.AUTO_PLACE_RIGHT;
                grabPos = ARM_POSITION.AUTO_GRAB_RIGHT;
            }
        } else {
            if (alliance == Alliance.Blue) {
                firstTraj = PathPlanner.loadPath("CloseToSubstation", new PathConstraints(3, 3));
                secondTraj = PathPlanner.loadPath("CloseToSubstationBack", new PathConstraints(3, 3));
                placePos = ARM_POSITION.AUTO_PLACE_RIGHT;
                grabPos = ARM_POSITION.AUTO_GRAB_RIGHT;
            } else {
                firstTraj = PathPlanner.loadPath("FarFromSubstation", new PathConstraints(3, 3));
                secondTraj = PathPlanner.loadPath("FarFromSubstationBack", new PathConstraints(3, 3));
                placePos = ARM_POSITION.AUTO_PLACE_LEFT;
                grabPos = ARM_POSITION.AUTO_GRAB_LEFT;
            }
        }

        addCommands(
            new InstantCommand(drivetrain::highGear),
            new ArmOutOfStart(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
            new MoveArmUnsafe(arm, placePos),
            new InstantCommand(arm::unclamp),
            new ParallelCommandGroup(
                new MoveArmUnsafe(arm, ARM_POSITION.HOME),
                new FollowPathPlannerTrajectory(drivetrain, firstTraj, true)
            ),
            new MoveArmUnsafe(arm, grabPos),
            new InstantCommand(arm::clamp),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
                new FollowPathPlannerTrajectory(drivetrain, secondTraj, false)
            ),
            new InstantCommand(arm::unclamp)
        );
    }
}
