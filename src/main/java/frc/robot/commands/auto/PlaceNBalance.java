// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.BackwardsBalance;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNBalance extends SequentialCommandGroup {
  /** Creates a new PlaceNBalance. */
  public PlaceNBalance(Drivetrain drivetrain, Arm arm) {
    addCommands(
      new InstantCommand(drivetrain::highGear),
      new ParallelCommandGroup(
        new EncoderDrive(-0.5, drivetrain),
        new ArmOutOfStart(arm)
      ),
      new ParallelCommandGroup(
        new EncoderDrive(0.5, drivetrain),
        new SequentialCommandGroup(
          new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
          new WaitCommand(0.5)
        )
      ),
      new InstantCommand(arm::unclamp),
      new WaitCommand(0.1),
      new ParallelCommandGroup(
        new BackwardsBalance(drivetrain),
        new MoveArmUnsafe(arm, ARM_POSITION.HOME)
      )
    );
  }
}
