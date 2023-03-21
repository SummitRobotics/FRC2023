package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.Ports;

public class ArmIntake extends SubsystemBase {

    @AutoLog
    public static class IntakeIOInputs {
        public double encoderPosition = 0.0;
        public double encoderVelocity = 0.0;
        public String state = "";
    }

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private final CANSparkMax motor = new CANSparkMax(Ports.Arm.INTAKE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private State state;
    private double otherSpeed;
    private RollingAverage current;
    private boolean lock = false;

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

    private final double INTAKE_SPEED = -0.7 * 12;
    private final double OUTTAKE_SPEED = 0.5 * 12;
    private final double STALL_SPEED = -0.6 * 12;
    private final double STALL_CURRENT = 70.0;

    public ArmIntake() {
        state = State.STATIONARY;
        otherSpeed = 0;
        current = new RollingAverage(10, false);
        current.update(0);
    }

    public void lock() {
        lock = true;
    }

    public void unlock() {
        lock = false;
    }
    // So this can be used in the end of a command.
    public void unlock(boolean nothing) {
        lock = false;
    }

    public boolean notLocked() {
        return !lock;
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
        if (state == State.OTHER) {
            motor.set(otherSpeed);
            return;
        }
        if (state == State.INTAKE && current.getAverage() > STALL_CURRENT) state = State.STALLING;

        double power = 0;

        if (state == State.INTAKE) {
            power = INTAKE_SPEED;
        } else if (state == State.OUTTAKE) {
            power = OUTTAKE_SPEED;
        } else if (state == State.STALLING) {
            power = STALL_SPEED;
        } else if (state == State.STATIONARY) {
            power = 0;
        }
        motor.setVoltage(power);
        // motor.set(state == State.INTAKE ? INTAKE_SPEED : state == State.OUTTAKE ? -INTAKE_SPEED :
        //     state == State.STALLING ? STALL_SPEED : state == State.STALLING ? 0 : otherSpeed);

        // Update logs
        updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);
    }

    /**
     * Stops the intake motor, setting the state to STATIONARY.
     */
    public void stop() {
        state = State.STATIONARY;
        motor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      // builder.addStringProperty("armConfiguration", getCurrentArmConfiguration()::toString, null);
      builder.addStringProperty("State", () -> getState().toString(), null);
      builder.addDoubleProperty("Current", current::getAverage, null);

    }

    private void updateInputs(IntakeIOInputs inputs) {
        inputs.encoderPosition = getEncoderPos();
        inputs.encoderVelocity = encoder.getVelocity();
        inputs.state = state.toString();
    }
}
