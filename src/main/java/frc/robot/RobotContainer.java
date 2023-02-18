// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.FullManualArm;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Ports;

public class RobotContainer {

  private CommandScheduler scheduler;
  private ControllerDriver driverXBox;
  private ControllerDriver gunnerXBox;
  private LaunchpadDriver launchpad;
  private Command teleopInit;
  private ArcadeDrive arcadeDrive;
  private Drivetrain drivetrain;
  private Arm arm;
  // private Intake intake;
  private AHRS gyro;


  public RobotContainer() {

    scheduler = CommandScheduler.getInstance();
    driverXBox = new ControllerDriver(Ports.OI.DRIVER_XBOX_PORT);
    gunnerXBox = new ControllerDriver(Ports.OI.GUNNER_XBOX_PORT);
    launchpad = new LaunchpadDriver(Ports.OI.LAUNCHPAD_PORT);
    gyro = new AHRS();
    drivetrain = new Drivetrain(gyro);
    arm = new Arm();
    // intake = new Intake();

    
    arcadeDrive = new ArcadeDrive(
      drivetrain,
      driverXBox.rightTrigger,
      driverXBox.leftTrigger,
      driverXBox.leftX,
      driverXBox.dPadAny
    );
    

    teleopInit = new SequentialCommandGroup(
      // new Home(intake),
      // new InstantCommand(() -> intake.lock())
    );

    setDefaultCommands();
    configureBindings();
    initTelemetry();
  }

  private void initTelemetry() {
    SmartDashboard.putData("Drivetrain", drivetrain);
  }

  private void configureBindings() {
    driverXBox.rightBumper.prioritize(AxisPriorities.DRIVE).getTrigger()
      .onTrue(new InstantCommand(drivetrain::highGear));
    driverXBox.leftBumper.prioritize(AxisPriorities.DRIVE).getTrigger()
      .onTrue(new InstantCommand(drivetrain::lowGear));
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
    drivetrain.setDefaultCommand(arcadeDrive);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit() {
    scheduler.schedule(teleopInit);
  }
}
