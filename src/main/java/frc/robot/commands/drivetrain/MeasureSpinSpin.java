// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MeasureSpinSpin extends SequentialCommandGroup {
  /** Creates a new MeasureSpinSpin. */

  List<SpinSpin> spins = new ArrayList<>();

  public MeasureSpinSpin(Drivetrain drivetrain, double start, double end, double inc) {

    for (double i = start; i <= end; i += inc) {
      SpinSpin spin = new SpinSpin(drivetrain, i);
      spins.add(spin);
      addCommands(
        spin,
        new WaitCommand(0.25)
      );
    }

    addCommands(new InstantCommand(() -> {
      System.out.println("---------- SPIN SPIN TEST OVER ----------");
      for (SpinSpin spin : spins) {
        System.out.println(spin.toString());
      }
    }));
  }

  public MeasureSpinSpin(Drivetrain drivetrain) {
    this(drivetrain, 0.05, 0.5, 0.05);
  }
}
