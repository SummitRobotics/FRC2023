// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Home;
// import frc.robot.commands.TimedMoveMotor;
// import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmOutOfStart extends SequentialCommandGroup {
  /** Creates a new ArmOutOfStart. */
  public ArmOutOfStart(Arm arm) {
    addCommands(
    new InstantCommand(() -> arm.manualHome(ARM_POSITION.HOME))
    // new TimedMoveMotor(arm::setJoint2MotorVoltage, 10, 1),
    // new MoveArmUnsafe(arm, ARM_POSITION.PRE_HOME, true),
    // new Home(arm),
    // new ParallelCommandGroup(
        // new TimedMoveMotor(arm::setJoint3MotorVoltage, -5, 0.2),
        // new TimedMoveMotor(arm::setJoint1MotorVoltage, 5, 0.1),
        // new TimedMoveMotor(arm::setJoint2MotorVoltage, 5, 0.1),
        // new TimedMoveMotor(arm::setTurretMotorVoltage, 5, 0.1)
    // )
    );
  }
}
