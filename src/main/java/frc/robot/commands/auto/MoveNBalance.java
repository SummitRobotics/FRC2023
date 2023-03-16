// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.ChargeBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.ChargeBalance.BalanceDirection;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class MoveNBalance extends SequentialCommandGroup {
  /** Creates a new MoveNBalance. */
  public MoveNBalance(Arm arm, Drivetrain drive) {
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new ArmOutOfStart(arm),
          new MoveArmUnsafe(arm, ARM_POSITION.HOME)
        ),
        new SequentialCommandGroup(
          new FunctionalCommand(() -> {drive.setBothMotorPower(-0.5);}, () -> {}, (bool) -> {drive.setBothMotorPower(0);}, () -> drive.gyro.getRoll() < -10, drive),
          new PrintCommand("tip detected, switching to encoder"),
          new EncoderDrive(-1.5, drive),
          new PrintCommand("balancing"),
          new ChargeBalance(drive, BalanceDirection.FORWARD)
        )
      )
    );
  }
}
