// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.utilities.Positions;

public class MovePositions extends CommandBase {
  /** Creates a new MovePositions. */

  enum LOCATION {
    MIDDLE_LOW(90.27,49.4,24.86,-3.21,-40.02),
    MIDDLE_MEDIUM(94.003,80.28,94.83,-20.33,-40.02),
    MIDDLE_HIGH(94.00,102.52,136.56,-36.07,-40.02),
    LEFT_LOW(0,0,0,0,-40.02),
    LEFT_MEDIUM(0,0,0,0,-40.02),
    LEFT_HIGH(0,0,0,0,-40.02),
    RIGHT_LOW(0,0,0,0,-40.02),
    RIGHT_MEDIUM(0,0,0,0,-40.02),
    RIGHT_HIGH(0,0,0,0,-40.02);

    public ArmConfiguration config;
    LOCATION(double turret, double joint1, double joint2, double joint3, double wrist) {
      config = new ArmConfiguration(turret, joint1, joint2, joint3, wrist, ArmConfiguration.POSITION_TYPE.ENCODER_ROTATIONS);
    }
  }

  LOCATION location;
  Arm arm;

  public MovePositions(Arm arm, LOCATION location) {
    this.arm = arm;
    this.location = location;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setToConfiguration(location.config);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atTargetConfiguration(0.01);
  }
}
