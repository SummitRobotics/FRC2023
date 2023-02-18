// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.utilities.Positions;

public class MoveArmUnsafe extends CommandBase {

  private final Arm arm;
  private final ArmConfiguration armConfiguration;

  public MoveArmUnsafe(Arm arm, Positions.Pose3d endPosition, double grabberAngleRadians, double wristRotationRadians) {
    this.arm = arm;
    this.armConfiguration = ArmConfiguration.fromEndPosition(endPosition, grabberAngleRadians, wristRotationRadians);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setToConfiguration(armConfiguration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atConfiguration(armConfiguration);
  }
}
