// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class DefaultArmCommand extends CommandBase {

  Arm arm;

  /** Creates a new DefaultArmCommand. */
  public DefaultArmCommand(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (arm.atConfiguration(ARM_POSITION.HOME.config, 0.05) || arm.getTargetArmConfiguration().equals(ARM_POSITION.HOME.config, 0.01)) {
      arm.setToConfiguration(ARM_POSITION.HOME.config);
    } else {
      arm.setToConfiguration(arm.getCurrentArmConfiguration());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
