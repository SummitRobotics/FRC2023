// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.LogComponents;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;

public class RobotContainer {

  private CommandScheduler scheduler;
  private Command teleopInit;

  // Devices
  private AHRS navx;

  // Subsystems
  private Drivetrain drivetrain;
  private Arm arm;

  public RobotContainer() {
    scheduler = CommandScheduler.getInstance();

    // Devices.
    navx = new AHRS();

    // Sybsystems
    drivetrain = Drivetrain.init(navx, new Pose2d());
    arm = new Arm();

    // Configure the bindings
    configureBindings();

    // Init Logging and Telemetry
    initLogging();
    initTelemetry();
  }

  private void configureBindings() {}

  private void initLogging() {
    scheduler.schedule(new LogComponents(drivetrain, arm));
  }

  private void initTelemetry() {
    Shuffleboard.getTab("Telemetry").add(arm);
    Shuffleboard.getTab("Telemetry").add(drivetrain);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit() {
    scheduler.schedule(teleopInit);
  }
}
