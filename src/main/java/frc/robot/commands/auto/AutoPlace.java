package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class AutoPlace extends SequentialCommandGroup {

    public AutoPlace(Arm arm, Drivetrain drivetrain) {
        addCommands(
            new ArmOutOfStart(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.RIGHT_HIGH),
            new WaitCommand(0.5),
            new InstantCommand(() -> arm.unclamp())
        );
    }
}
