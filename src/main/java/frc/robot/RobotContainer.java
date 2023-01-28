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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.devices.AprilTagCameraWraper;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain drivetrain;
  private final AHRS gyro;
  private final Command arcadeDrive;
  private final ControllerDriver controller;

  private final AprilTagCameraWraper camera1;
  private final AprilTagCameraWraper camera2;
  private final AprilTagCameraWraper camera3;

  public RobotContainer() throws IOException {
    controller = new ControllerDriver(0);
    gyro = new AHRS();
    drivetrain = new Drivetrain(gyro);

    camera1 = new AprilTagCameraWraper("Camera1", new Transform3d(new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(11.625), Units.inchesToMeters(32.5)), new Rotation3d(0,0,Math.PI/2)));
    camera2 = new AprilTagCameraWraper("Camera2", new Transform3d(new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(3), Units.inchesToMeters(27.25)), new Rotation3d(0,0,0)));
    camera3 = new AprilTagCameraWraper("Camera3", new Transform3d(new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(-11.625), Units.inchesToMeters(32.5)), new Rotation3d(0,0,-Math.PI/2)));

    drivetrain.addVisionCamera(camera1);
    drivetrain.addVisionCamera(camera2);
    drivetrain.addVisionCamera(camera3);

    arcadeDrive = new ArcadeDrive(drivetrain, controller.rightTrigger, controller.leftTrigger, controller.leftX, controller.dPadAny);
    drivetrain.setDefaultCommand(arcadeDrive);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
