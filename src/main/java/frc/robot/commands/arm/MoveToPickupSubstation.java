// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.FieldElementPositions;

public class MoveToPickupSubstation extends CommandBase {

  public enum Side {
    LEFT, RIGHT
  }

  private final Arm arm;
  private final Side side;

  private ArmConfiguration armConfigurationInit;
  private ArmConfiguration armConfigurationFinal;

  private Timer timer;

  /** Creates a new MoveToPickupSubstation. */
  public MoveToPickupSubstation(Arm arm, Side side) {
    this.arm = arm;
    this.side = side;
    timer = new Timer();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance alliance = DriverStation.getAlliance();
    Translation3d moveTo = new Translation3d(10, 10, 10);

    if (alliance == Alliance.Blue) {
      if (side == Side.LEFT) {
        moveTo = FieldElementPositions.BLUE_LEFT_SUBSTATION;
      }
      if (side == Side.RIGHT) {
        moveTo = FieldElementPositions.BLUE_RIGHT_SUBSTATION;
      }
    } else if (alliance == Alliance.Red) {
      if (side == Side.LEFT) {
        moveTo = FieldElementPositions.RED_LEFT_SUBSTATION;
      }
      if (side == Side.RIGHT) {
        moveTo = FieldElementPositions.RED_RIGHT_SUBSTATION;
      }
    }

    ArmConfiguration currentPos = arm.getCurrentArmConfiguration();
    armConfigurationFinal = ArmConfiguration.fromEndPosition(Positions.Pose3d.fromFieldSpace(moveTo), 0, 0);
    armConfigurationInit = new ArmConfiguration(armConfigurationFinal.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS), currentPos.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), currentPos.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), currentPos.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), currentPos.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS), POSITION_TYPE.ENCODER_ROTATIONS);

    arm.setToConfiguration(armConfigurationInit);
    timer.stop();
    timer.reset();
  }

  @Override
  public void execute() {
    if (arm.atConfiguration(armConfigurationInit, 0.02)) {
      timer.start();
    }

    if (timer.hasElapsed(1)) {
      arm.setToConfiguration(armConfigurationFinal);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armConfigurationFinal = null;
    armConfigurationInit = null;
    if (interrupted) {
      arm.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (armConfigurationFinal == null) {
      return false;
    }
    return arm.atConfiguration(armConfigurationFinal, 0.02) || !armConfigurationFinal.validConfig(arm.getCurrentArmConfiguration()) || !armConfigurationInit.validConfig(arm.getCurrentArmConfiguration());
  }
}
