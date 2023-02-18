// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utilities.Positions;

public class MoveArmHome extends SequentialCommandGroup {
  /** Creates a new MoveArmHome. */
  // TODO Find Home Position
  public MoveArmHome(Arm arm) {
    addCommands(new MoveArm(arm, Positions.Pose3d.fromRobotSpace(new Translation3d(0, 0, 0)), 0, 0));
  }
}
