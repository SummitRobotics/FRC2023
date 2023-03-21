// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.devices.LEDs.LEDCalls;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions.ARM_POSITION;

public class MovePositionsLaunchpad extends CommandBase {
  enum HEIGHT {
    LOW, MEDIUM, HIGH, NONE
  }

  enum POSITION {
    LEFT, MIDDLE, RIGHT, NONE
  }

  Arm arm;
  LaunchpadDriver launchpadDriver;
  RobotContainer robotContainer;

  HEIGHT height;
  HEIGHT activeLED;

  POSITION position;
  POSITION prevPosition;

  boolean shouldClamp = true;


  /** Creates a new MovePositionsLaunchpad. */
  public MovePositionsLaunchpad(Arm arm, LaunchpadDriver launchpad, RobotContainer robotContainer) {

    this.arm = arm;
    this.launchpadDriver = launchpad;
    this.robotContainer = robotContainer;
    this.height = HEIGHT.NONE;
    this.activeLED = HEIGHT.NONE;
    this.position = POSITION.NONE;
    prevPosition = POSITION.NONE;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotContainer.enableAltMode();
    arm.setToConfiguration(ARM_POSITION.HOME.config);
    this.height = HEIGHT.NONE;
    this.position = POSITION.NONE;
    launchpadDriver.buttonB.setLED(false);
    launchpadDriver.buttonE.setLED(false);
    launchpadDriver.buttonH.setLED(false);
    launchpadDriver.buttonC.setLED(false);
    launchpadDriver.buttonF.setLED(false);
    launchpadDriver.buttonI.setLED(false);
    launchpadDriver.buttonA.setLED(true);

    if (Arm.LIDAR_CLAMP_NEAR < arm.getLidarDistance() && arm.getLidarDistance() < Arm.LIDAR_CLAMP_FAR) {
      shouldClamp = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // System.out.println(arm.getClampSolenoidState());
    // System.out.println(shouldClamp);
    // System.out.println(arm.getLidarDistance());
    if (arm.getClampSolenoidState() && shouldClamp && Arm.LIDAR_CLAMP_NEAR < arm.getLidarDistance() && arm.getLidarDistance() < Arm.LIDAR_CLAMP_FAR) {
      // arm.clamp();
      shouldClamp = false;
    }
    if (arm.getClampSolenoidState() && !shouldClamp && (Arm.LIDAR_CLAMP_NEAR > arm.getLidarDistance() || arm.getLidarDistance() > Arm.LIDAR_CLAMP_FAR)) {
      shouldClamp = true;
    }

    if (prevPosition != position) {
      if (position != POSITION.NONE) {
        LEDCalls.CARDIANL_SELECT.activate();
      }
      prevPosition = position;
    }

    if (height == HEIGHT.NONE && activeLED != HEIGHT.NONE) {
      LEDCalls.ARM_LOW.cancel();
      LEDCalls.ARM_MID.cancel();
      LEDCalls.ARM_HIGH.cancel();
      activeLED = HEIGHT.NONE;
    }

    if (height == HEIGHT.LOW && activeLED != HEIGHT.LOW) {
      LEDCalls.ARM_LOW.activate();
      LEDCalls.ARM_MID.cancel();
      LEDCalls.ARM_HIGH.cancel();
      activeLED = HEIGHT.LOW;
    }

    if (height == HEIGHT.MEDIUM && activeLED != HEIGHT.MEDIUM) {
      LEDCalls.ARM_LOW.cancel();
      LEDCalls.ARM_MID.activate();
      LEDCalls.ARM_HIGH.cancel();
      activeLED = HEIGHT.MEDIUM;
    }

    if (height == HEIGHT.HIGH && activeLED != HEIGHT.HIGH) {
      LEDCalls.ARM_LOW.cancel();
      LEDCalls.ARM_MID.cancel();
      LEDCalls.ARM_HIGH.activate();
      activeLED = HEIGHT.HIGH;
    }

    if (launchpadDriver.buttonC.getTrigger().getAsBoolean()) {
      this.height = HEIGHT.HIGH;
      launchpadDriver.buttonC.setLED(true);
      launchpadDriver.buttonF.setLED(false);
      launchpadDriver.buttonI.setLED(false);
      launchpadDriver.buttonA.setLED(false);
    }

    if (launchpadDriver.buttonF.getTrigger().getAsBoolean()) {
      this.height = HEIGHT.MEDIUM;
      launchpadDriver.buttonC.setLED(false);
      launchpadDriver.buttonF.setLED(true);
      launchpadDriver.buttonI.setLED(false);
      launchpadDriver.buttonA.setLED(false);
    }

    if (launchpadDriver.buttonI.getTrigger().getAsBoolean()) {
      this.height = HEIGHT.LOW;
      launchpadDriver.buttonC.setLED(false);
      launchpadDriver.buttonF.setLED(false);
      launchpadDriver.buttonI.setLED(true);
      launchpadDriver.buttonA.setLED(false);
    }

    if (launchpadDriver.buttonB.getTrigger().getAsBoolean()) {
      this.position = POSITION.LEFT;
      launchpadDriver.buttonB.setLED(true);
      launchpadDriver.buttonE.setLED(false);
      launchpadDriver.buttonH.setLED(false);
      launchpadDriver.buttonA.setLED(false);
    }

    if (launchpadDriver.buttonE.getTrigger().getAsBoolean()) {
      this.position = POSITION.MIDDLE;
      launchpadDriver.buttonB.setLED(false);
      launchpadDriver.buttonE.setLED(true);
      launchpadDriver.buttonH.setLED(false);
      launchpadDriver.buttonA.setLED(false);
    }

    if (launchpadDriver.buttonH.getTrigger().getAsBoolean()) {
      this.position = POSITION.RIGHT;
      launchpadDriver.buttonB.setLED(false);
      launchpadDriver.buttonE.setLED(false);
      launchpadDriver.buttonH.setLED(true);
      launchpadDriver.buttonA.setLED(false);
    }

    if (launchpadDriver.buttonA.getTrigger().getAsBoolean()) {
      this.position = POSITION.NONE;
      this.height = HEIGHT.NONE;
      launchpadDriver.buttonB.setLED(false);
      launchpadDriver.buttonE.setLED(false);
      launchpadDriver.buttonH.setLED(false);
      launchpadDriver.buttonC.setLED(false);
      launchpadDriver.buttonF.setLED(false);
      launchpadDriver.buttonI.setLED(false);
      launchpadDriver.buttonA.setLED(true);
    }

    if (height == HEIGHT.NONE || position == POSITION.NONE) {
      arm.setToConfiguration(ARM_POSITION.HOME.config);
    } else if (height == HEIGHT.LOW) {
      if (position == POSITION.LEFT) {
        arm.setToConfiguration(ARM_POSITION.LEFT_LOW.config);
      } else if (position == POSITION.MIDDLE) {
        arm.setToConfiguration(ARM_POSITION.GROUND_PICKUP_QUORB.config);
      } else if (position == POSITION.RIGHT) {
        arm.setToConfiguration(ARM_POSITION.RIGHT_LOW.config);
      }
    } else if (height == HEIGHT.MEDIUM) {
      if (position == POSITION.LEFT) {
        arm.setToConfiguration(ARM_POSITION.LEFT_MEDIUM.config);
      } else if (position == POSITION.MIDDLE) {
        arm.setToConfiguration(ARM_POSITION.MIDDLE_MEDIUM.config);
      } else if (position == POSITION.RIGHT) {
        arm.setToConfiguration(ARM_POSITION.RIGHT_MEDIUM.config);
      }
    } else if (height == HEIGHT.HIGH) {
      if (position == POSITION.LEFT) {
        arm.setToConfiguration(ARM_POSITION.LEFT_HIGH.config);
      } else if (position == POSITION.MIDDLE) {
        arm.setToConfiguration(ARM_POSITION.MIDDLE_HIGH.config);
      } else if (position == POSITION.RIGHT) {
        arm.setToConfiguration(ARM_POSITION.RIGHT_HIGH.config);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launchpadDriver.buttonC.setLED(false);
    launchpadDriver.buttonF.setLED(false);
    launchpadDriver.buttonI.setLED(false);
    launchpadDriver.buttonB.setLED(false);
    launchpadDriver.buttonE.setLED(false);
    launchpadDriver.buttonH.setLED(false);
    launchpadDriver.buttonA.setLED(false);
    robotContainer.disableAltMode();
    arm.stop();
    LEDCalls.ARM_LOW.cancel();
    LEDCalls.ARM_MID.cancel();
    LEDCalls.ARM_HIGH.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
