// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.devices.AprilTagCameraWraper;
import frc.robot.devices.PCM;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.Ports;

public class RobotContainer {

  private final Drivetrain drivetrain;
  private final AHRS gyro;
  private final Command arcadeDrive;
  private final ControllerDriver controller;
  private final PCM pcm;
  private CommandScheduler scheduler;
  
  private Command teleInit;

  // private final AprilTagCameraWraper camera1;
  // private final AprilTagCameraWraper camera2;
  // private final AprilTagCameraWraper camera3;

  public RobotContainer() throws IOException {
    scheduler = CommandScheduler.getInstance();

    controller = new ControllerDriver(Ports.OI.XBOX_PORT);
    gyro = new AHRS();
    drivetrain = new Drivetrain(gyro);
    pcm = new PCM(Ports.Other.PCM, drivetrain);


    // camera1 = new AprilTagCameraWraper("Camera1", new Transform3d(new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(11.625), Units.inchesToMeters(32.5)), new Rotation3d(0,0,Math.PI/2)));
    // camera2 = new AprilTagCameraWraper("Camera2", new Transform3d(new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(3), Units.inchesToMeters(27.25)), new Rotation3d(0,0,0)));
    // camera3 = new AprilTagCameraWraper("Camera3", new Transform3d(new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(-11.625), Units.inchesToMeters(32.5)), new Rotation3d(0,0,-Math.PI/2)));

    // drivetrain.addVisionCamera(camera1);
    // drivetrain.addVisionCamera(camera2);
    // drivetrain.addVisionCamera(camera3);

    arcadeDrive = new ArcadeDrive(drivetrain, controller.rightTrigger, controller.leftTrigger, controller.leftX, controller.dPadAny);
    drivetrain.setDefaultCommand(arcadeDrive);

    teleInit = new SequentialCommandGroup(
      new InstantCommand(pcm::enableCompressorDigital),
      new InstantCommand(drivetrain::highGear)
    );

    SmartDashboard.putData(drivetrain);

    configureBindings();
  }

  private void configureBindings() {
    controller.buttonA.getTrigger().onTrue(new InstantCommand(drivetrain::toggleShift));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void robotInit() {
    gyro.calibrate();
  }

  public void teleInit() {
    scheduler.schedule(teleInit);
  }
}
