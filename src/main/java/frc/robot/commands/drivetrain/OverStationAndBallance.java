// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.auto.ArmOutOfStart;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OverStationAndBallance extends SequentialCommandGroup {
  /** Creates a new OverStationAndBallance. */
  public OverStationAndBallance(Arm arm, Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new ArmOutOfStart(arm),
          new MoveArmUnsafe(arm, ARM_POSITION.HOME)
        ),
        new SequentialCommandGroup(
          new FunctionalCommand(() -> {drive.setBothMotorPower(-0.5);}, () -> {}, (bool) -> {drive.setBothMotorPower(0);}, () -> drive.gyro.getRoll() < -10, drive),
          new PrintCommand("tip detected, switching to encoder"),
          new EncoderDrive(-1.5, -1.5, drive),
          new PrintCommand("balancing"),
          new ChargeStationBalance(drive)
        )
      )
    );
  }
}
