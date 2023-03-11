package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.Ports;

public class ArmIntake extends SubsystemBase {
    
    private final CANSparkMax motor = new CANSparkMax(Ports.Arm.INTAKE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private State state;
    private double otherSpeed;
    private RollingAverage current;

    public enum State {
        INTAKE,
        OUTTAKE,
        STATIONARY,
        STALLING,
        OTHER;

        public String toString() {
            return this == INTAKE ? "Intake" : this == OUTTAKE ? "Outtake" :
                this == STATIONARY ? "Stationary" : this == STALLING ? "Stalling" : "Other";
        }
    }

    private final double INTAKE_SPEED = 0.5;
    private final double STALL_SPEED = 0.2;
    private final double STALL_CURRENT = 15.0;

    public ArmIntake() {
        state = State.STATIONARY;
        otherSpeed = 0;
        current = new RollingAverage(10, true);
    }

    /**
     * Sets the state of the arm-mounted intake.
     * @param state The state to set the intake to.
     */
    public void setState(State state) {
        this.state = state;
    }

    /**
     * Gets the current state of the arm-mounted intake.
     * @return The current intake state
     */
    public State getState() {
        return state;
    }

    /**
     * Sets the speed of the intake motor, switching the state to OTHER.
     * @param speed The speed to set the intake motor to.
     */
    public void setSpeed(double speed) {
        otherSpeed = speed;
        state = State.OTHER;
    }

    /**
     * Gets the current encoder position of the intake motor.
     *
     * @return The current encoder position of the intake motor.
     */
    public double getEncoderPos() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        current.update(motor.getOutputCurrent());
        if (current.getAverage() > STALL_CURRENT) state = State.STALLING;
        motor.set(state == State.INTAKE ? INTAKE_SPEED : state == State.OUTTAKE ? -INTAKE_SPEED :
            state == State.STALLING ? STALL_SPEED : state == State.STALLING ? 0 : otherSpeed);
    }

    /**
     * Stops the intake motor, setting the state to STATIONARY.
     */
    public void stop() {
        state = State.STATIONARY;
        motor.set(0);
    }
}
