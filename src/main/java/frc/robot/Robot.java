// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    try {
      m_robotContainer = new RobotContainer();
    } catch (IOException e) {
      e.printStackTrace();
      System.exit(1);
    }
    m_robotContainer.robotInit();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  @Override
  public void disabledExit() {
    m_robotContainer.disabledExit();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.autonomousPeriodic();
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.autonomousExit();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopPeriodic();
  }

  @Override
  public void teleopExit() {
    m_robotContainer.teleopExit();
  }

  @Override
  public void testInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.testInit();
  }
  @Override
  public void testPeriodic() {
    m_robotContainer.testPeriodic();
  }

  @Override
  public void testExit() {
    m_robotContainer.testExit();
  }
}
