package frc.robot.commands.auto;

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
import frc.robot.commands.drivetrain.ChargeBalance.BalanceDirection;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmIntake.INTAKE_ELEMENT_TYPE;
import frc.robot.subsystems.arm.ArmIntake.State;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNMoveNGrabNBalance extends SequentialCommandGroup {
  
    public PlaceNMoveNGrabNBalance(Arm arm, ArmIntake armIntake, Drivetrain drivetrain) {

        addCommands(
            new InstantCommand(() -> armIntake.setState(State.STATIONARY)),
            new InstantCommand(() -> armIntake.setType(INTAKE_ELEMENT_TYPE.CONE)),
            new InstantCommand(() -> armIntake.setState(State.STALLING)),
            new ArmOutOfStart(arm),
            new MoveArmUnsafe(arm, ARM_POSITION.COBRA_START, false, false),
            new MoveArmUnsafe(arm, ARM_POSITION.ONE_POINT_FIVE_PLACE),
            // new WaitCommand(0.25),
            new EjectElement(armIntake),
            new InstantCommand(drivetrain::lowGear),
            new InstantCommand(() -> AutoPickup.setType(ELEMENT_TYPE.CONE)),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new MoveArmUnsafe(arm, ARM_POSITION.HOME),
                    new MoveArmUnsafe(arm, ARM_POSITION.ONE_POINT_FIVE_INTERMEDIATE, false, false),
                    new ParallelRaceGroup(
                        new AutoPickup(arm, armIntake, drivetrain, LOCATION.ONE_POINT_FIVE),
                        new WaitCommand(3.65)
                    ),
                    new MoveArmUnsafe(arm, ARM_POSITION.HOME)
                ),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new InstantCommand(() -> drivetrain.setBothMotorPower(-0.5), drivetrain).repeatedly(),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 8),
                            new WaitCommand(0.5),
                            new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) < 8),
                            new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 10)
                        )
                    ),
                    new InstantCommand(() -> drivetrain.setBothMotorPower(0), drivetrain),
                    new EncoderDrive(-2.5, -2.5, drivetrain, 0.5),
                    new EncoderDrive(2.5, 2.5, drivetrain, 0.5)
                )
            ),
            new ChargeBalance(drivetrain, BalanceDirection.FORWARD)
        );
    }
}
