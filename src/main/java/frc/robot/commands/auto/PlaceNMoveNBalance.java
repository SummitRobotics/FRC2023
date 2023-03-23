package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.EjectElement;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.ChargeBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.ChargeBalance.BalanceDirection;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmIntake.INTAKE_ELEMENT_TYPE;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNMoveNBalance extends SequentialCommandGroup {
    public PlaceNMoveNBalance(Arm arm, ArmIntake armIntake, Drivetrain drivetrain) {
        addCommands(
            new InstantCommand(() -> armIntake.setState(State.STATIONARY)),
            new InstantCommand(() -> armIntake.setType(INTAKE_ELEMENT_TYPE.QUORB)),
            new InstantCommand(() -> armIntake.setState(State.STALLING)),
            new InstantCommand(drivetrain::highGear),
            new ArmOutOfStart(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
            // 0.4748 is the distance from charge station to nodes minus our bumper length
            // new ParallelCommandGroup(
                // new EncoderDrive(0.4748, drivetrain),
                // new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH)
            // ),
            new EncoderDrive(0.4748, drivetrain),
            new EjectElement(armIntake),
            new WaitCommand(0.25),
            new InstantCommand(drivetrain::lowGear),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    new MoveArmUnsafe(arm, ARM_POSITION.HOME)
                ),
                new ParallelRaceGroup(
                    new InstantCommand(() -> drivetrain.setBothMotorPower(-0.5), drivetrain).repeatedly(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 8),
                        new WaitCommand(0.5),
                        new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) < 8),
                        new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 10)
                    )
                )
            ),
            new InstantCommand(() -> drivetrain.setBothMotorPower(0), drivetrain),
            new EncoderDrive(-1, drivetrain),
            new WaitCommand(1.25),
            new ChargeBalance(drivetrain, BalanceDirection.FORWARD)
        );
    }
}
