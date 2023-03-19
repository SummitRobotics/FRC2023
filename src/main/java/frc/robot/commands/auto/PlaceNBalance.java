// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Home;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.ChargeBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.ChargeBalance.BalanceDirection;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNBalance extends SequentialCommandGroup {
  /** Creates a new PlaceNBalance. */
  public PlaceNBalance(Drivetrain drivetrain, Arm arm) {
    addCommands(
      new InstantCommand(drivetrain::highGear),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.25),
          new ArmOutOfStart(arm),
          new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_MEDIUM),
          new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH)
        ),
        new EncoderDrive(-0.5, drivetrain)
      ),
      new EncoderDrive(0.5, drivetrain),
      new WaitCommand(0.2),
      new InstantCommand(arm::unclamp),
      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new ChargeBalance(drivetrain, BalanceDirection.BACKWARD),
        new SequentialCommandGroup(
          new MoveArmUnsafe(arm, ARM_POSITION.PRE_HOME),
          new Home(arm),
          new MoveArmUnsafe(arm, ARM_POSITION.HOME)
        )
      )
    );
  }
}
