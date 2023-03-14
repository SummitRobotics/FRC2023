// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automovements;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmUnsafe;
import frc.robot.commands.drivetrain.EncoderDrive;
import frc.robot.commands.drivetrain.MoveToElement;
import frc.robot.devices.LEDs.LEDCalls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickup extends SequentialCommandGroup {
  /** Creates a new AutoPickup. */
  public AutoPickup(Drivetrain drivetrain, Arm arm, PhotonCamera quorbCamera, PhotonCamera coneCamera) {
    addCommands(
      new InstantCommand(LEDCalls.INTAKE_DOWN::activate),
      new InstantCommand(arm::unclamp),
      new MoveArmUnsafe(arm, ARM_POSITION.GROUND_PICKUP),
      new WaitCommand(0.25),
      new MoveToElement(drivetrain, quorbCamera, coneCamera).until(() -> {
        System.out.println(arm.getLidarDistance());
        return false;
      }),
      new EncoderDrive(0.15, 0.15, drivetrain),
      new InstantCommand(arm::clamp),
      new WaitCommand(0.75),
      new MoveArmUnsafe(arm, ARM_POSITION.GROUND_PICKUP_SAFE),
      new MoveArmUnsafe(arm, ARM_POSITION.HOME)
    );

    // TODO - fix crash caused by this block
    // handleInterrupt(() -> {
      // LEDCalls.INTAKE_DOWN.cancel();
      // arm.setToConfiguration(ARM_POSITION.HOME.config);
    // });
  }
}
