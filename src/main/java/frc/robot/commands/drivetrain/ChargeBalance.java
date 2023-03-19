// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeBalance extends SequentialCommandGroup {

  public enum BalanceDirection {
    FORWARD, BACKWARD
  }

  private final static double
    POWER_INIT = 0.5,
    POWER_RAMP = 0.2,
    POWER_TUNE = 0.2;
    
  public ChargeBalance(Drivetrain drivetrain, BalanceDirection direction) {
    this (drivetrain, () -> direction);
  }

  public ChargeBalance(Drivetrain drivetrain, Supplier<BalanceDirection> direction) {
    addCommands(
      new InstantCommand(drivetrain::lowGear),
      new ParallelRaceGroup(
        new SelectCommand(Map.ofEntries(
          Map.entry(BalanceDirection.FORWARD, new InstantCommand(() -> drivetrain.setBothMotorPower(POWER_INIT), drivetrain).repeatedly()),
          Map.entry(BalanceDirection.BACKWARD, new InstantCommand(() -> drivetrain.setBothMotorPower(-POWER_INIT), drivetrain).repeatedly())
        ), direction::get),
        new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) > 12)
      ),
      new WaitCommand(0.5),
      new ParallelRaceGroup(
        new SelectCommand(Map.ofEntries(
          Map.entry(BalanceDirection.FORWARD, new InstantCommand(() -> drivetrain.setBothMotorPower(POWER_RAMP), drivetrain).repeatedly()),
          Map.entry(BalanceDirection.BACKWARD, new InstantCommand(() -> drivetrain.setBothMotorPower(-POWER_RAMP), drivetrain).repeatedly())
        ), direction::get),
        new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) < 12)
      ),
      new ParallelRaceGroup(
        new SelectCommand(Map.ofEntries(
          Map.entry(BalanceDirection.FORWARD, new InstantCommand(() -> drivetrain.setBothMotorPower(-POWER_TUNE), drivetrain).repeatedly()),
          Map.entry(BalanceDirection.BACKWARD, new InstantCommand(() -> drivetrain.setBothMotorPower(POWER_TUNE), drivetrain).repeatedly())
        ), direction::get),
        new WaitUntilCommand(() -> Math.abs(drivetrain.gyro.getRoll()) < 2)
      ),
      new JustBalance(drivetrain),
      new InstantCommand(drivetrain::stop, drivetrain),
      new InstantCommand(drivetrain::highGear)
      );
  }
}
