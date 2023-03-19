// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.ChangeRateLimiter;

public class SpinSpin extends CommandBase {

  private final Drivetrain drivetrain;
  private final Timer timer;
  private final ChangeRateLimiter limiter;

  private final double power;
  private double rate;
  private boolean measured = false;
  private boolean end = false;

  private static final double DELAY = 0.75;
  private static final double RATE = 0.02;


  /** Creates a new SpinSpin. */
  public SpinSpin(Drivetrain drivetrain, double power) {
    this.drivetrain = drivetrain;
    timer = new Timer();
    this.power = power;
    limiter = new ChangeRateLimiter(RATE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    measured = false;
    end = false;
    limiter.resetOld();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setPower = 0;
    if (!measured) {
      setPower = limiter.getRateLimitedValue(this.power);
      if (timer.get() > DELAY) {
        rate = drivetrain.gyro.getRate();
        measured = true;
      }
      if (setPower == power) {
        timer.start();
      }
    } else {
      setPower = limiter.getRateLimitedValue(0);
      if (setPower == 0) {
        end = true;
      }
    }

    drivetrain.setLeftMotorPower(setPower);
    drivetrain.setRightMotorPower(-setPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setBothMotorPower(0);
    if (!interrupted) {
      System.out.println(prettyString());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }

  @Override
  public String toString() {
    //return String.format("SPIN MEASUREMENT - Power: %.2f Rate: %.2f", power, rate);
    return String.format("%.2f,%.2f", power, rate);
  }
  
  public String prettyString() {
    return String.format("SPIN MEASUREMENT - Power: %.2f Rate: %.2f", power, rate);
  }
}
