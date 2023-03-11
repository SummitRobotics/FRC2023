// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.FieldElementPositions;

public class MoveArmToNode extends CommandBase {

  private final Arm arm;

  private Positions.Pose3d node;
  private ArmConfiguration armConfiguration;

  StringSubscriber subscriber;

  /** Creates a new MoveArmToNode. */
  public MoveArmToNode(Arm arm) {
    this.arm = arm;
    addRequirements(arm);

    subscriber = NetworkTableInstance.getDefault().getTable("customDS").getStringTopic("location").subscribe("00");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final int value = Integer.parseInt(subscriber.get());

    System.out.println(subscriber.get());

    System.out.println(value);

    final int xCoordIndex = (value - value % 10) / 10;
    final int yCoordIndex = value % 10;

    if (xCoordIndex != 0 && yCoordIndex != 0) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        node = Positions.Pose3d.fromFieldSpace(new Translation3d(
            FieldElementPositions.BLUE_X_VALUES[xCoordIndex - 1],
            FieldElementPositions.BLUE_Y_VALUES[yCoordIndex - 1],
            FieldElementPositions.BLUE_Z_VALUES[xCoordIndex - 1]
        ));
      } else if (DriverStation.getAlliance() == Alliance.Red) { 
        node = Positions.Pose3d.fromFieldSpace(new Translation3d(
            FieldElementPositions.RED_X_VALUES[xCoordIndex - 1],
            FieldElementPositions.RED_Y_VALUES[yCoordIndex - 1],
            FieldElementPositions.RED_Z_VALUES[xCoordIndex - 1]
        ));
        System.out.println(node.inRobotSpace());
      }
    } else {
      node = Positions.Pose3d.fromFieldSpace(new Translation3d(10, 10, 10));
    }

    if (xCoordIndex == 3) {
      armConfiguration = ArmConfiguration.fromEndPosition(node, - (Math.PI / 4));
    } else {
      armConfiguration = ArmConfiguration.fromEndPosition(node, 0);
    }

    System.out.println(armConfiguration);

    arm.setToConfiguration(armConfiguration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      arm.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("here");
    if (armConfiguration == null) {
      return false;
    }
    return arm.atConfiguration(armConfiguration, 0.02) || !armConfiguration.validConfig(arm.getCurrentArmConfiguration());
  }
}
