// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.FullManualArm;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Ports;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.LogComponents;

public class RobotContainer {

  private CommandScheduler scheduler;
  private NetworkTableInstance networktable;

  private ControllerDriver driverXBox;
  private ControllerDriver gunnerXBox;
  private LaunchpadDriver launchpad;

  private Command teleopInit;
  // private ArcadeDrive arcadeDrive;
  // private Drivetrain drivetrain;
  private Arm arm;
  
  // private AHRS navx;

  public RobotContainer() {
    scheduler = CommandScheduler.getInstance();
    networktable = NetworkTableInstance.getDefault();

    // OI
    driverXBox = new ControllerDriver(Ports.OI.DRIVER_XBOX_PORT);
    gunnerXBox = new ControllerDriver(Ports.OI.GUNNER_XBOX_PORT);
    launchpad = new LaunchpadDriver(Ports.OI.LAUNCHPAD_PORT);

    // Devices
    // navx = new AHRS();

    // Subsystems
    // drivetrain = Drivetrain.init(navx, new Pose2d());
    arm = new Arm();
    
    // Commands
    // arcadeDrive = new ArcadeDrive(
    //   drivetrain,
    //   driverXBox.rightTrigger,
    //   driverXBox.leftTrigger,
    //   driverXBox.leftX,
    //   driverXBox.dPadAny
    // );

    setDefaultCommands();

    // Configure the bindings
    configureBindings();

    // Init Logging and Telemetry
    initLogging();
    initTelemetry();
  }

  private void configureBindings() {
    // driverXBox.rightBumper.prioritize(AxisPriorities.DRIVE).getTrigger()
    //   .onTrue(new InstantCommand(drivetrain::highGear));
    // driverXBox.leftBumper.prioritize(AxisPriorities.DRIVE).getTrigger()
    //   .onTrue(new InstantCommand(drivetrain::lowGear));
    launchpad.buttonC.getTrigger()
      .toggleOnTrue(new FullManualArm(arm, FullManualArm.Type.TURRET, gunnerXBox));
    launchpad.buttonB.getTrigger()
      .toggleOnTrue(new FullManualArm(arm, FullManualArm.Type.JOINT_1, gunnerXBox));
    launchpad.buttonA.getTrigger()
      .toggleOnTrue(new FullManualArm(arm, FullManualArm.Type.JOINT_2, gunnerXBox));
    launchpad.buttonF.getTrigger()
      .toggleOnTrue(new FullManualArm(arm, FullManualArm.Type.JOINT_3, gunnerXBox));
    launchpad.buttonE.getTrigger()
      .toggleOnTrue(new FullManualArm(arm, FullManualArm.Type.WRIST, gunnerXBox));
    
  }

  private void setDefaultCommands() {
    // drivetrain.setDefaultCommand(arcadeDrive);
  }

  private void initLogging() {
    scheduler.schedule(new LogComponents(arm));
    // scheduler.schedule(new LogComponents(drivetrain, arm));

  }

  private void initTelemetry() {
    SmartDashboard.putData("Arm", arm);
    // SmartDashboard.putData("Drivetrain", drivetrain);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit() {
    scheduler.schedule(teleopInit);
  }
}
