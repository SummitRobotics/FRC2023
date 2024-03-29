// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;
import frc.robot.utilities.Positions;

public class MoveArmUnsafe extends CommandBase {

  private final Arm arm;
  private ArmConfiguration armConfiguration;

  private Positions.Pose3d pose;
  private double grabberAngle;

  private boolean fromPose = false;
  private boolean extraUnsafe = false;

  private boolean homeMove = false;
  private boolean precise = true;

  public MoveArmUnsafe(Arm arm, Positions.Pose3d endPosition, double grabberAngleRadians) {
    this.arm = arm;
    this.pose = endPosition;
    this.grabberAngle = grabberAngleRadians;
    this.fromPose = true;
    addRequirements(arm);
  }

  public MoveArmUnsafe(Arm arm, ArmConfiguration armConfiguration) {
    this.arm = arm;
    this.armConfiguration = armConfiguration;
    addRequirements(arm);
  }

  public MoveArmUnsafe(Arm arm, ARM_POSITION location) {
    this(arm, location.config);
    if (location == ARM_POSITION.HOME) {
      homeMove = true;
    }
  }

  public MoveArmUnsafe(Arm arm, ARM_POSITION location, boolean extraUnsafe) {
    this(arm, location.config);
    this.extraUnsafe = extraUnsafe;
  }

  public MoveArmUnsafe(Arm arm, ARM_POSITION location, boolean extraUnsafe, boolean isPrecise) {
    this(arm, location, extraUnsafe);
    precise = isPrecise;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("MOVETHING");
    if (homeMove) {
      Arm.setDistanceCheck(false);
    }
    if (armConfiguration == null || fromPose) {
      System.out.println(pose.inRobotSpace());
      armConfiguration = ArmConfiguration.fromEndPosition(pose, grabberAngle);
    }
    // System.out.println(armConfiguration);
    if (extraUnsafe) {
      arm.setToConfigurationUnsafe(armConfiguration);
    } else {
      arm.setToConfiguration(armConfiguration);
    }
    // System.out.println("MoveArmUnsafe: " + armConfiguration);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("HERE1");
    Arm.setDistanceCheck(true);
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atConfiguration(armConfiguration, precise ? 0.02 : 0.1) || !armConfiguration.validConfig(arm.getCurrentArmConfiguration());
  }
}
