// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;

public class JustBalance extends CommandBase {

  private final double
  P = 0.005,
  I = 0, // do not change this
  D = 0;

  private final Drivetrain drivetrain;
  private final AHRS gyro;
  private PIDController controller;

  public JustBalance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.gyro = drivetrain.gyro;
    this.controller = new PIDController(P, I, D);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    controller.setTolerance(0.25, 1);
    controller.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -Functions.clampDouble(
                controller.calculate(-gyro.getRoll()),
                0.75,
                -0.75);
            drivetrain.setBothMotorPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    drivetrain.highGear();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
