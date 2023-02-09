// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {

  private final Arm arm;
  private final Translation3d endPosition;
  private final double grabberAngleRadians;
  private final double wristRotationRadians;

  private List<Translation3d> path;
  private SequentialCommandGroup moveCommand;

  public MoveArm(Arm arm, Translation3d endPosition, double grabberAngleRadians, double wristRotationRadians) {
    this.arm = arm;
    this.endPosition = endPosition;
    this.grabberAngleRadians = grabberAngleRadians;
    this.wristRotationRadians = wristRotationRadians;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path = Arm.MovementMap.generatePathBetweenTwoPoints(arm.getCurrentArmConfiguration().getEndPosition().getTranslation(), endPosition);

    if (path != null) {
      List<Command> commands = new ArrayList<>();
  
      for (Translation3d point : path) {
        commands.add(new MoveArmUnsafe(arm, point, grabberAngleRadians, wristRotationRadians));
      }
  
      moveCommand = new SequentialCommandGroup(commands.toArray(new Command[0]));
      moveCommand.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (moveCommand != null) {
      moveCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (moveCommand != null) {
      moveCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (moveCommand != null) {
      return moveCommand.isFinished();
    } else {
      return true;
    }
  }
}
