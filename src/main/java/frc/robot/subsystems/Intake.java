package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.homing.HomeableCANSparkMax;
import frc.robot.utilities.homing.HomeableSubsystem;
import frc.robot.utilities.lists.Ports;

public class Intake extends SubsystemBase implements HomeableSubsystem, Loggable {

    private final CANSparkMax intakeMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final CANSparkMax pivotMotor = new CANSparkMax(Ports.Intake.PIVOT_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private final Solenoid lock = new Solenoid(PneumaticsModuleType.REVPH, Ports.Intake.LOCK_SOLENOID);
    private boolean isLocked;
    private State state;

    public enum State {
        LOWERED_MOVING,
        LOWERED_STATIONARY, // this currently isn't used
        RAISED;

        public String toString() {
            if (this == LOWERED_MOVING) {
                return "Lowered Moving";
            } else if (this == LOWERED_STATIONARY) {
                return "Lowered Stationary";
            } else {
                return "Raised";
            }
        }
    }

    private final double INTAKE_SPEED = 0.5;

    /**
     * Creates a new Intake.
     */
    public Intake() {
        state = State.RAISED;
        isLocked = true;
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
     * @param state
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

    /**
     * Sets the lock state; true is locked.
     *
     * @param isLocked
     */
    public void setLock(boolean isLocked) {
        this.isLocked = isLocked;
        lock.set(isLocked);
    }

    /**
     * Locks the pneumatic piston.
     */
    public void lock() {
        setLock(true);
    }

    /**
     * Unlocks the pneumatic piston.
     */
    public void unlock() {
        setLock(false);
    }

    /**
     * Toggles the pneumatic piston.
     */
    public void toggleLock() {
        setLock(!isLocked);
    }

    public double getPivotEncoderPos() {
        return pivotEncoder.getPosition();
    }

    @Override
    public HomeableCANSparkMax[] getHomeables() {
        // TODO - check if motor power should be positive or negative, check current threshold
        return new HomeableCANSparkMax[] {
            new HomeableCANSparkMax(
                intakeMotor,
                this,
                0.3,
                15.0
            )
        };
    }

    @Override
    public Subsystem getSubsystemObject() {
        return this;
    }

    @Override
    public void periodic() {
        setIntakeMotor(state == State.LOWERED_MOVING ? INTAKE_SPEED : 0);
    }

    @Override
    public String getLogName() {
        return "Intake";
    }

    @Override
    public HashMap<String, Supplier<String>> getStringLogData() {
        HashMap<String, Supplier<String>> out = new HashMap<>();
        out.put("Intake Piston", getState()::toString);
        return out;
    }

    @Override
    public HashMap<String, DoubleSupplier> getDoubleLogData() {
        HashMap<String, DoubleSupplier> out = new HashMap<>();
        out.put("Intake Motor Velocity", intakeEncoder::getVelocity);
        out.put("Pivot Motor Position", pivotEncoder::getPosition);
        return out;
    }
}
