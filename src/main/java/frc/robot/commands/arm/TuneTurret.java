// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OITrigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;

public class TuneTurret extends CommandBase {

  private final Arm arm;
  private final OITrigger lefTrigger;
  private final OITrigger rightTrigger;
  private ArmConfiguration initalConfiguration;
  private double turretOffset = 0;

  /** Creates a new TuneTurret. */
  public TuneTurret(Arm arm, OITrigger lefTrigger, OITrigger rightTrigger) {
    this.arm = arm;
    this.lefTrigger = lefTrigger;
    this.rightTrigger = rightTrigger;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initalConfiguration = arm.getCurrentArmConfiguration();
    turretOffset = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initalConfiguration == null) return;
    if (lefTrigger.get()) {
      turretOffset -= 0.01;
    } else if (rightTrigger.get()) {
      turretOffset += 0.01;
    }
    ArmConfiguration newArmConfig = new ArmConfiguration(
      initalConfiguration.getTurretPosition(ArmConfiguration.POSITION_TYPE.ANGLE) + turretOffset,
      initalConfiguration.getFirstJointPosition(ArmConfiguration.POSITION_TYPE.ANGLE),
      initalConfiguration.getSecondJointPosition(ArmConfiguration.POSITION_TYPE.ANGLE),
      initalConfiguration.getThirdJointPosition(ArmConfiguration.POSITION_TYPE.ANGLE), 
      ArmConfiguration.POSITION_TYPE.ANGLE);
    arm.setToConfiguration(newArmConfig);
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
