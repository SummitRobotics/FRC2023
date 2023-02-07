package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Ports;

public class NewIntake extends SubsystemBase /*implements Loggable, HomeableSubsystem*/ {
    private final CANSparkMax intakeMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final CANSparkMax pivotMotor = new CANSparkMax(Ports.Intake.PIVOT_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private final Solenoid lock = new Solenoid(PneumaticsModuleType.REVPH, Ports.Intake.LOCK_SOLENOID);
    private State state;

    public enum State {
        LOWERED_MOVING,
        LOWERED_STATIONARY,
        RAISED;
    }

    /**
     * Creates a new Intake.
     */
    public NewIntake() {
        state = State.RAISED;
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
     * Sets the pivot motor to a certain speed.
     *
     * @param speed The speed to set the motor to.
     */
    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * Sets the intake state
     *
     * @param state The intake state
     */
    public void setState(State state) {
        this.state = state;
    }

    /*
     * Returns the current state of the intake.
     */
    public State getState() {
        return state;
    }

    public void setLock(boolean isLocked) {
        
    }
}
