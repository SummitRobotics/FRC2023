package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmIntake.INTAKE_ELEMENT_TYPE;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.commands.arm.EjectElement;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.automovements.AutoPickup;
import frc.robot.commands.automovements.AutoPickup.ELEMENT_TYPE;
import frc.robot.commands.automovements.AutoPickup.LOCATION;
import frc.robot.commands.drivetrain.FollowPathPlannerTrajectory;
import frc.robot.subsystems.Drivetrain;

public class PlaceNMoveNGrabNPlace extends SequentialCommandGroup {

    public enum Type {
        Blue,
        Red
    }

    public PlaceNMoveNGrabNPlace(Arm arm, ArmIntake armIntake, Drivetrain drivetrain, Type type) {

        PathPlannerTrajectory firstTraj;
        PathPlannerTrajectory secondTraj;

        if (type == Type.Blue) {
            firstTraj = PathPlanner.loadPath("Blue", new PathConstraints(2, 2), true);
            secondTraj = PathPlanner.loadPath("BlueBack", new PathConstraints(2, 2), true);
        } else {
            firstTraj = PathPlanner.loadPath("Red", new PathConstraints(3, 3));
            secondTraj = PathPlanner.loadPath("RedBack", new PathConstraints(3, 3));
        }

        addCommands(
            new InstantCommand(() -> armIntake.setState(State.STATIONARY)),
            new InstantCommand(() -> armIntake.setType(INTAKE_ELEMENT_TYPE.CONE)),
            new InstantCommand(() -> armIntake.setState(State.STALLING)),
            new InstantCommand(drivetrain::highGear),
            new ArmOutOfStart(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.COBRA_START, false, false),
            new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
            new EjectElement(armIntake),
            new InstantCommand(() -> AutoPickup.setType(ELEMENT_TYPE.QUORB)),
            new InstantCommand(() -> armIntake.setType(INTAKE_ELEMENT_TYPE.QUORB)),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new MoveArmUnsafe(arm, ARM_POSITION.HOME),
                    new ParallelRaceGroup(
                        new AutoPickup(arm, armIntake, drivetrain, LOCATION.GROUND),
                        new WaitCommand(3.65)
                    ),
                    new MoveArmUnsafe(arm, ARM_POSITION.HOME),
                    new MoveArmUnsafe(arm, ARM_POSITION.COBRA_START, false, false),
                    new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH)
                ),
                new SequentialCommandGroup(
                    new FollowPathPlannerTrajectory(drivetrain, firstTraj, true),
                    new FollowPathPlannerTrajectory(drivetrain, secondTraj, true)
                )
            ),
            new EjectElement(armIntake)
        );
    }
}
