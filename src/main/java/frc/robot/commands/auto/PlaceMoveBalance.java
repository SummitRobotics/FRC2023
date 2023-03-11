package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.ChargeStationBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceMoveBalance extends SequentialCommandGroup {
    public PlaceMoveBalance(Arm arm, Drivetrain drivetrain) {
        addCommands(
            new InstantCommand(drivetrain::highGear),
            new ArmOutOfStart(arm),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
                    new WaitCommand(0.25)
                ),
                // 0.4748 is the distance from charge station to nodes minus our bumper length
                new EncoderDrive(0.4748, drivetrain)
            ),
            new InstantCommand(arm::unclamp),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    // drive over the station
                    new FunctionalCommand(
                        () -> drivetrain.setBothMotorPower(-0.5),
                        () -> {},
                        (interrupted) -> drivetrain.stop(),
                        () -> drivetrain.gyro.getRoll() < -10,
                        drivetrain
                    ),
                    // forward and balance
                    new EncoderDrive(0.75, drivetrain),
                    new ChargeStationBalance(drivetrain)
                ),
                new MoveArmUnsafe(arm, ARM_POSITION.HOME)
            )
        );
    }
}
