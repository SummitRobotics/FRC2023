package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.EjectElement;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.automovements.AutoPickup;
import frc.robot.commands.automovements.AutoPickup.ELEMENT_TYPE;
import frc.robot.commands.automovements.AutoPickup.LOCATION;
import frc.robot.commands.drivetrain.ChargeBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.TurnByEncoder;
import frc.robot.commands.drivetrain.ChargeBalance.BalanceDirection;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmIntake.INTAKE_ELEMENT_TYPE;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNMoveNGrabNBalance extends SequentialCommandGroup {
  
    public PlaceNMoveNGrabNBalance(Arm arm, ArmIntake armIntake, Drivetrain drivetrain, Alliance alliance) {
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
            // new MoveArmUnsafe(arm, ARM_POSITION.COBRA_START),
            new MoveArmUnsafe(arm, ARM_POSITION.HOME),
            new InstantCommand(drivetrain::lowGear),
            new ParallelRaceGroup(
                new InstantCommand(() -> drivetrain.setBothMotorPower(-0.5), drivetrain).repeatedly(),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 8),
                    new WaitCommand(0.5),
                    new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) < 8),
                    new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 10)
                )
                // new WaitCommand(2.1)
            ),
            new InstantCommand(() -> drivetrain.setBothMotorPower(0), drivetrain),
            new EncoderDrive(-1.3, -1.3, drivetrain, 0.5),
            new WaitCommand(0.25),
            new TurnByEncoder(180, drivetrain),
            new WaitCommand(0.25),
            new InstantCommand(armIntake::stop),
            new InstantCommand(() -> AutoPickup.setType(ELEMENT_TYPE.CONE)),
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    new AutoPickup(arm, armIntake, drivetrain, LOCATION.GROUND),
                    new WaitCommand(2)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(2),
                    new EncoderDrive(1.5, 1.5, drivetrain, 0.5)
                )
            ),
            new MoveArmUnsafe(arm, ARM_POSITION.HOME),
            new ChargeBalance(drivetrain, BalanceDirection.BACKWARD)
        );
    }
}
