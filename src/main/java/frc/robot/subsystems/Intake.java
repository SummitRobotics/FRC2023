// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.lists.Ports;

public class Intake extends SubsystemBase implements Loggable{

  private final CANSparkMax intakeMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private final Solenoid intakePiston = new Solenoid(PneumaticsModuleType.REVPH, Ports.Intake.INTAKE_SOLENOID);
  private IntakeState intakeState;

  /**
   * Enum for the intake state
   */
  public enum IntakeState {
    LOWERED(false),
    RAISED(true);

    public final boolean value;
    IntakeState(boolean value) {
      this.value = value;
    }

    public static IntakeState fromBoolean(boolean value) {
      return value == RAISED.value ? RAISED : LOWERED;
    }

    public IntakeState toggle() {
      return this == RAISED ? LOWERED : RAISED;
    }
  }

  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakeState = IntakeState.fromBoolean(intakePiston.get());
  }

  /**
   * Sets the intake motor to a certain speed.
   * 
   * @param speed The speed to set the motor to.
   */
  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Sets the Intake Piston to a State.
   */
  public void setIntakeState(IntakeState state) {
    intakePiston.set(state.value);
    intakeState = state;
  }

  /**
   * Toggles the intake piston.
   */
  public void toggleIntakePiston() {
    setIntakeState(intakeState.toggle());
  }

  /**
   * Returns the current intake state.
   */
  public IntakeState getIntakeState() {
    return intakeState;
  }

  /*
  Below are the methods for the Loggable interface.
   */
  @Override
  public String getLogName() {
    return "Intake";
  }

  @Override
  public HashMap<String, BooleanSupplier> getBooleanLogData() {
    HashMap<String, BooleanSupplier> out = new HashMap<>();
    out.put("Intake Piston", () -> getIntakeState().value);
    return out;
  }

  @Override
  public HashMap<String, DoubleSupplier> getDoubleLogData() {
    HashMap<String, DoubleSupplier> out = new HashMap<>();
    out.put("Intake Motor Velocity", intakeEncoder::getVelocity);
    return out;
    }
}
