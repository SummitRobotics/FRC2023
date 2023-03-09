// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MoveToElement extends CommandBase {
  /** Creates a new AutoPickup. */

  private final Drivetrain drivetrain;
  private final PhotonCamera quorbCamera, coneCamera;

  private PhotonCamera trackingCamera;

  private PIDController pidController = new PIDController(0.01, 0, 0);

  private Timer resulTimeout = new Timer();

  private boolean end = false;

  private static final double 
    TIMEOUT = 1,
    DRIVE_POWER = 0.2;

  public MoveToElement(Drivetrain drivetrain, PhotonCamera quorbCamera, PhotonCamera coneCamera) {
    this.drivetrain = drivetrain;
    this.quorbCamera = quorbCamera;
    this.coneCamera = coneCamera;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PhotonPipelineResult quorbResult = quorbCamera.getLatestResult();
    PhotonPipelineResult coneResult = coneCamera.getLatestResult();

    if (quorbResult.hasTargets() || coneResult.hasTargets()) {
      if (!quorbResult.hasTargets() || coneResult.getBestTarget().getPitch() < quorbResult.getBestTarget().getPitch()) {
        trackingCamera = coneCamera;
      } else {
        trackingCamera = quorbCamera;
      }
    }

    resulTimeout.reset();
    resulTimeout.start();

    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (trackingCamera == null) return;
    PhotonPipelineResult result = trackingCamera.getLatestResult();
    if (!result.hasTargets()) return;
    if (result.getBestTarget().getArea() > 0.2) end = true;
    resulTimeout.restart();

    double turningPower = pidController.calculate(result.getBestTarget().getYaw(), 0);

    drivetrain.setLeftMotorPower(DRIVE_POWER + turningPower);
    drivetrain.setRightMotorPower(DRIVE_POWER - turningPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    resulTimeout.reset();
    resulTimeout.stop();
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end || resulTimeout.get() > TIMEOUT;
  }
}
