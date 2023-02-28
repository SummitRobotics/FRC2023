// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;

public class MovePositionsLaunchpad extends CommandBase {

  enum LOCATION {
    MIDDLE_LOW(90.27,49.4,24.86,-3.21,-40.02),
    MIDDLE_MEDIUM(94.003,80.28,94.83,-20.33,-40.02),
    MIDDLE_HIGH(94.00,102.52,136.56,-36.07,-40.02),
    LEFT_LOW(34.9282,46.3091,12.000,-8.26,-50.2618),
    LEFT_MEDIUM(34.9282,9.7619,60.70,-39.9520,-50.2618),
    LEFT_HIGH(34.9282,76.8835,121.43,-51.1905,-50.2618),
    RIGHT_LOW(150.71,46.3091,12.000,-8.26,-50.2618),
    RIGHT_MEDIUM(155.471,9.7619,60.70,-39.9520,-50.2618),
    RIGHT_HIGH(155.471,76.8835,121.43,-51.1905,-50.2618),
    HOME(95.5, 2, 2, -14.5, -41);

    public ArmConfiguration config;
    LOCATION(double turret, double joint1, double joint2, double joint3, double wrist) {
      config = new ArmConfiguration(turret, joint1, joint2, joint3, wrist, ArmConfiguration.POSITION_TYPE.ENCODER_ROTATIONS);
    }
  }

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
  POSITION position;


  /** Creates a new MovePositionsLaunchpad. */
  public MovePositionsLaunchpad(Arm arm, LaunchpadDriver launchpad, RobotContainer robotContainer) {

    this.arm = arm;
    this.launchpadDriver = launchpad;
    this.robotContainer = robotContainer;
    this.height = HEIGHT.NONE;
    this.position = POSITION.NONE;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotContainer.enableAltMode();
    arm.setToConfiguration(LOCATION.HOME.config);
    this.height = HEIGHT.NONE;
    this.position = POSITION.NONE;
    launchpadDriver.buttonB.setLED(false);
    launchpadDriver.buttonE.setLED(false);
    launchpadDriver.buttonH.setLED(false);
    launchpadDriver.buttonC.setLED(false);
    launchpadDriver.buttonF.setLED(false);
    launchpadDriver.buttonI.setLED(false);
    launchpadDriver.buttonA.setLED(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
      arm.setToConfiguration(LOCATION.HOME.config);
    } else if (height == HEIGHT.LOW) {
      if (position == POSITION.LEFT) {
        arm.setToConfiguration(LOCATION.LEFT_LOW.config);
      } else if (position == POSITION.MIDDLE) {
        arm.setToConfiguration(LOCATION.MIDDLE_LOW.config);
      } else if (position == POSITION.RIGHT) {
        arm.setToConfiguration(LOCATION.RIGHT_LOW.config);
      }
    } else if (height == HEIGHT.MEDIUM) {
      if (position == POSITION.LEFT) {
        arm.setToConfiguration(LOCATION.LEFT_MEDIUM.config);
      } else if (position == POSITION.MIDDLE) {
        arm.setToConfiguration(LOCATION.MIDDLE_MEDIUM.config);
      } else if (position == POSITION.RIGHT) {
        arm.setToConfiguration(LOCATION.RIGHT_MEDIUM.config);
      }
    } else if (height == HEIGHT.HIGH) {
      if (position == POSITION.LEFT) {
        arm.setToConfiguration(LOCATION.LEFT_HIGH.config);
      } else if (position == POSITION.MIDDLE) {
        arm.setToConfiguration(LOCATION.MIDDLE_HIGH.config);
      } else if (position == POSITION.RIGHT) {
        arm.setToConfiguration(LOCATION.RIGHT_HIGH.config);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
