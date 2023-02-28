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
  private ArmConfiguration armConfiguration;

  private Positions.Pose3d pose;
  private double grabberAngle;
  private double wristAngle;

  private boolean fromPose = false;

  public MoveArmUnsafe(Arm arm, Positions.Pose3d endPosition, double grabberAngleRadians, double wristRotationRadians) {
    this.arm = arm;
    this.pose = endPosition;
    this.grabberAngle = grabberAngleRadians;
    this.wristAngle = wristRotationRadians;
    this.fromPose = true;
    addRequirements(arm);
  }

  public MoveArmUnsafe(Arm arm, ArmConfiguration armConfiguration) {
    this.arm = arm;
    this.armConfiguration = armConfiguration;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("MOVETHING");
    if (armConfiguration == null || fromPose) {
      System.out.println(pose.inRobotSpace());
      armConfiguration = ArmConfiguration.fromEndPosition(pose, grabberAngle, wristAngle);
    }
    arm.setToConfiguration(armConfiguration);
    // System.out.println("MoveArmUnsafe: " + armConfiguration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("HERE1");
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atConfiguration(armConfiguration, 0.05) || !armConfiguration.validConfig(arm.getCurrentArmConfiguration());
  }
}
