package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.ChargeStationBalance;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class Auto extends SequentialCommandGroup {

    public Auto(Arm arm, Drivetrain drivetrain, Trajectory trajectory, boolean chrageStation) {
        addCommands(
            new ArmOutOfStart(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.RIGHT_HIGH),
            new WaitCommand(0.5),
            new InstantCommand(() -> arm.unclamp()),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_MEDIUM),
                    new MoveArmUnsafe(arm, ARM_POSITION.HOME)
                ), 
                new FollowTrajectory(drivetrain, trajectory)
            ),
            chrageStation ? new ChargeStationBalance(drivetrain) : new InstantCommand()
        );
    }
}
