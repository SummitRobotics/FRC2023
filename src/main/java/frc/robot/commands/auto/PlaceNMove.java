// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.EjectElement;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class PlaceNMove extends SequentialCommandGroup {
  /** Creates a new PlaceNMove. */
  public PlaceNMove(Drivetrain drivetrain, ArmIntake armIntake, Arm arm) {
    addCommands(
      new InstantCommand(drivetrain::highGear),
      new ParallelCommandGroup(
        new EncoderDrive(-0.75, drivetrain),
        new ArmOutOfStart(arm)
      ),
      new MoveArmUnsafe(arm, ARM_POSITION.MIDDLE_HIGH),
      new EncoderDrive(0.75, drivetrain),
      new WaitCommand(0.25),
      new EjectElement(armIntake),
      new WaitCommand(0.25),
      new ParallelCommandGroup(
        new EncoderDrive(-4, drivetrain),
        new MoveArmUnsafe(arm, ARM_POSITION.HOME)
      )
    );
  }
}
