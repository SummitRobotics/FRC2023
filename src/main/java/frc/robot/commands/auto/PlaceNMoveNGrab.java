package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.EjectElement;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.automovements.AutoPickup;
import frc.robot.commands.automovements.AutoPickup.ELEMENT_TYPE;
import frc.robot.commands.automovements.AutoPickup.LOCATION;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmIntake.INTAKE_ELEMENT_TYPE;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNMoveNGrab extends SequentialCommandGroup {
    public PlaceNMoveNGrab(Arm arm, Drivetrain drivetrain, ArmIntake armIntake) {
        addCommands(
            new InstantCommand(() -> armIntake.setState(State.STATIONARY)),
            new InstantCommand(() -> armIntake.setType(INTAKE_ELEMENT_TYPE.CONE)),
            new InstantCommand(() -> armIntake.setState(State.STALLING)),
            new InstantCommand(drivetrain::highGear),
            new ArmOutOfStart(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.COBRA_START),
            new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
            new WaitCommand(0.25),
            new EjectElement(armIntake),
            new ParallelCommandGroup(
              new SequentialCommandGroup(
                new WaitCommand(1),
                new EncoderDrive(-3, -3, drivetrain, 0.5)
              ),
              new MoveArmUnsafe(arm, ARM_POSITION.HOME)
            ),
            new TurnByEncoder(180, drivetrain),
            new InstantCommand(armIntake::stop),
            new InstantCommand(() -> AutoPickup.setType(ELEMENT_TYPE.CONE)),
            new ParallelCommandGroup(
              new AutoPickup(arm, armIntake, drivetrain, LOCATION.GROUND),
              new SequentialCommandGroup(
                new WaitCommand(2),
                new EncoderDrive(2, 2, drivetrain, 0.5)
              )
            )
            // new Home(arm.getHomeables()[1], arm.getHomeables()[2], arm.getHomeables()[3]),
            // new MoveArmUnsafe(arm, ARM_POSITION.HOME)
        );
    }
}
