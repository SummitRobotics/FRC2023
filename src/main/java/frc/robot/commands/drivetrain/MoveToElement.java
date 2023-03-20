// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.automovements.AutoPickup.ELEMENT_TYPE;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;

public class MoveToElement extends CommandBase {
  /** Creates a new AutoPickup. */

  private final Drivetrain drivetrain;
  private final PhotonCamera camera;

  private PIDController pidController = new PIDController(0.0035, 0, 0.0008);

  private Timer resulTimeout = new Timer();

  private ELEMENT_TYPE type;

  private double sizeThreshold;

  private static final double 
    TIMEOUT = 1,
    DRIVE_POWER = 0.4;

  public MoveToElement(Drivetrain drivetrain, PhotonCamera camera, ELEMENT_TYPE type) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    this.type = type;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (type == ELEMENT_TYPE.CONE) {
      sizeThreshold = 35;
    } else {
      sizeThreshold = 35;
    }

    resulTimeout.reset();
    resulTimeout.start();
    pidController.reset();
    pidController.setTolerance(8, 1);
    pidController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (camera == null) return;
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return;
    resulTimeout.restart();

    double yaw = result.getBestTarget().getYaw();

    double turningPower = pidController.calculate(yaw);
    double drivePower = DRIVE_POWER * (-0.005 * Math.abs(Math.pow(yaw, 2)) + 1);
    drivePower = drivePower * (-Math.abs(Math.pow((result.getBestTarget().getArea()/(sizeThreshold-5)), 1)) + 1);
    if (drivePower < 0) {
      drivePower = 0;
    }

    // drivePower = 0;

    drivetrain.setLeftMotorPower(drivePower + turningPower);
    drivetrain.setRightMotorPower(drivePower - turningPower);
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
    // System.out.println(end);
    // System.out.println(resulTimeout.get());
    return resulTimeout.get() > TIMEOUT;
  }
}
