// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automovements;

import java.time.Instant;
import java.util.Map;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
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

  public enum ELEMENT_TYPE {
    CONE,
    QUARB
  }

  private static ELEMENT_TYPE TYPE = ELEMENT_TYPE.CONE;

  public static ELEMENT_TYPE toggleType () {
    if (TYPE == ELEMENT_TYPE.CONE) {
      TYPE = ELEMENT_TYPE.QUARB;
    } else {
      TYPE = ELEMENT_TYPE.CONE;
    }
    return TYPE;
  }

  public static ELEMENT_TYPE getType () {
    return TYPE;
  }

  public static boolean isCone() {
    return TYPE == ELEMENT_TYPE.CONE;
  }

  public static boolean isQuorb() {
    return TYPE == ELEMENT_TYPE.QUARB;
  }

  /** Creates a new AutoPickup. */
  public AutoPickup(Drivetrain drivetrain, Arm arm, PhotonCamera grabberCam) {
    addCommands(
        new InstantCommand(() -> {
          if (getType() == ELEMENT_TYPE.CONE) {
            grabberCam.setPipelineIndex(0);
          } else {
            grabberCam.setPipelineIndex(1);
          }
        }),
        new InstantCommand(LEDCalls.INTAKE_DOWN::activate),
        new InstantCommand(arm::unclamp),
        new SelectCommand(Map.ofEntries(
          Map.entry(ELEMENT_TYPE.CONE, new MoveArmUnsafe(arm, ARM_POSITION.GROUND_PICKUP_CONE)),
          Map.entry(ELEMENT_TYPE.QUARB, new MoveArmUnsafe(arm, ARM_POSITION.GROUND_PICKUP_QUORB))
        ), () -> getType()),
        new WaitCommand(0.25),
        new MoveToElement(drivetrain, grabberCam, getType())
    );
  }
}
