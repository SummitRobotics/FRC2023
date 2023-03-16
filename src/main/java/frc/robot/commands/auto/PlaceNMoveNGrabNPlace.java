package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
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

    public PlaceNMoveNGrabNPlace(Arm arm, Drivetrain drivetrain, Type type) {

        PathPlannerTrajectory firstTraj;

        PathPlannerTrajectory secondTraj;
        
        ARM_POSITION placePos;
        ARM_POSITION grabPos;

        if (type == Type.FarFromSubstation) {
            firstTraj = PathPlanner.loadPath("FarFromSubstation", new PathConstraints(2, 2));
            secondTraj = PathPlanner.loadPath("FarFromSubstationBack", new PathConstraints(2, 2));
            if (DriverStation.getAlliance() == Alliance.Blue) {
                placePos = ARM_POSITION.AUTO_PLACE_FAR_BLUE;
                grabPos = ARM_POSITION.AUTO_GRAB_FAR_BLUE;
            } else {
                placePos = ARM_POSITION.RIGHT_HIGH;
                grabPos = ARM_POSITION.AUTO_GRAB_FAR_BLUE;
            }
        } else {
            firstTraj = PathPlanner.loadPath("CloseToSubstation", new PathConstraints(2, 2));
            secondTraj = PathPlanner.loadPath("CloseToSubstationBack", new PathConstraints(2, 2));
            if (DriverStation.getAlliance() == Alliance.Blue) {
                placePos = ARM_POSITION.RIGHT_HIGH;
                grabPos = ARM_POSITION.AUTO_GRAB_FAR_BLUE;
            } else {
                placePos = ARM_POSITION.AUTO_PLACE_FAR_BLUE;
                grabPos = ARM_POSITION.AUTO_GRAB_FAR_BLUE;
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
