// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.oi.inputs.OITrigger;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();

    // OITrigger oitrigger = new OITrigger(() -> true);
    // oitrigger.prioritize(5).getTrigger().onTrue(
        // new InstantCommand(() -> System.out.println("Test")));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
