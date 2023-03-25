package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.Ports;

public class ArmIntake extends SubsystemBase {

    // private final CANSparkMax motor = new CANSparkMax(Ports.Arm.INTAKE_MOTOR, MotorType.kBrushless);
    private final TalonFX motor = new TalonFX(Ports.Arm.INTAKE_MOTOR);


    private State state;
    private State oldState = state;
    private INTAKE_ELEMENT_TYPE type = INTAKE_ELEMENT_TYPE.CONE;

    private double otherSpeed;
    private RollingAverage current;
    private Timer stallTimerRampup = new Timer();
    private Timer stallTimer = new Timer();

    private boolean overheated = false;

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

    public final static double STALL_TIME = 0.25;

    public enum INTAKE_ELEMENT_TYPE {
        CONE(-1, 0.3, -0.6, 80.0, 0.75, 5.0),
        QUORB(0.5, -0.4, 0.2, 30.0, 0.75, 5.0);

        public final double intakeSpeed;
        public final double outtakeSpeed;
        public final double stallSpeed;
        public final double stallCurrent;
        public final double minStallTime;
        public final double thresholdCurrent;

        INTAKE_ELEMENT_TYPE(double intakeSpeed, double outtakeSpeed, double stallSpeed, double thresholdCurrent, double minStallTime, double stallCurrent) {
            this.intakeSpeed = intakeSpeed;
            this.outtakeSpeed = outtakeSpeed;
            this.stallSpeed = stallSpeed;
            this.stallCurrent = stallCurrent;
            this.minStallTime = minStallTime;
            this.thresholdCurrent = thresholdCurrent;
        }
    }

    // private final double INTAKE_SPEED = -1;
    // private final double OUTTAKE_SPEED = 0.5;
    // private final double STALL_SPEED = -0.6;
    // private final double STALL_CURRENT = 50.0;

    public ArmIntake() {
        state = State.STATIONARY;
        otherSpeed = 0;
        current = new RollingAverage(10, false);
        current.update(0);
        // motor.setSmartCurrentLimit((int) type.stallCurrent);
        // motor.setIdleMode(IdleMode.kBrake);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.currentLimit = (int) type.thresholdCurrent;
        config.supplyCurrLimit.enable = true;

        motor.configFactoryDefault();
        motor.configAllSettings(config);

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(12.0);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General,5);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

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
        // System.out.println(state);
        return state;
    }

    public boolean setType(INTAKE_ELEMENT_TYPE type) {
        if (state != State.STATIONARY) return false;

        this.type = type;
        return true;
    }

    /**
     * Sets the speed of the intake motor, switching the state to OTHER.
     * @param speed The speed to set the intake motor to.
     */
    public void setSpeed(double speed) {
        otherSpeed = speed;
        state = State.OTHER;
    }

    @Override
    public void periodic() {
        // current.update(motor.getOutputCurrent());
        current.update(motor.getSupplyCurrent());
        if (state ==  State.STALLING) {
            if (state != oldState) {
                // System.out.println("STALLING SETTING CURRENT");
                motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, (int) type.stallCurrent, (int) type.stallCurrent, 0));
                oldState = state;
            }
        } else {
            if (state != oldState) {
                // System.out.println("NOT STALLING SETTING CURRENT");
                motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, (int) type.thresholdCurrent, (int) type.thresholdCurrent, 0));
                oldState = state;
            }
        }

        if (state == State.INTAKE) {
            stallTimerRampup.start();
        } else {
            stallTimerRampup.stop();
            stallTimerRampup.reset();
        }

        // if (!overheated && motor.getTemperature() > 105) overheated = true;
        // if (overheated && motor.getTemperature() < 95) overheated = false;

        // if (!DriverStation.isFMSAttached() && overheated) {
        //     state = State.STATIONARY;
        //     motor.set(TalonFXControlMode.PercentOutput, 0);
        //     return;
        // }

        // if (!DriverStation.isFMSAttached() && motor.getTemperature() > 100) {
        //     motor.set(TalonFXControlMode.PercentOutput, INTAKE_SPEED);
        //     return;
        // }

        // if (state == State.OTHER) {
        //     motor.set(otherSpeed);
        //     return;
        // }

        if (state == State.OTHER) {
            motor.set(TalonFXControlMode.PercentOutput, otherSpeed);
            return;
        }

        if (state == State.INTAKE && stallTimerRampup.get() > type.minStallTime && current.getAverage() > type.thresholdCurrent) {
            if (stallTimer.get() == 0) {
                stallTimer.start();
            }
            if (stallTimer.get() > STALL_TIME) {
                state = State.STALLING;
                stallTimer.stop();
                stallTimer.reset();
            }
        } else if (stallTimer.get() != 0) {
            stallTimer.stop();
            stallTimer.reset();
        }

        double power = 0;
        if (state == State.INTAKE) {
            power = type.intakeSpeed;
        } else if (state == State.OUTTAKE) {
            power = type.outtakeSpeed;
        } else if (state == State.STALLING) {
            power = type.stallSpeed;
        } else if (state == State.STATIONARY) {
            power = 0;
        }

        // motor.setVoltage(power * 12);
        motor.set(TalonFXControlMode.PercentOutput, power); 
    }

    /**
     * Stops the intake motor, setting the state to STATIONARY.
     */
    public void stop() {
        state = State.STATIONARY;
        motor.set(TalonFXControlMode.PercentOutput, 0);
        // motor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      // builder.addStringProperty("armConfiguration", getCurrentArmConfiguration()::toString, null);
      builder.addStringProperty("State", () -> getState().toString(), null);
      builder.addDoubleProperty("Current", current::getAverage, null);
    //   builder.addDoubleProperty("MotorTemp-C", motor::getMotorTemperature, null);
    //   builder.addDoubleProperty("MotorTemp-F", () -> ((motor.getMotorTemperature() * 1.8) + 32), null);
    builder.addDoubleProperty("MotorTemp-C", motor::getTemperature, null);
    builder.addDoubleProperty("MotorTemp-F", () -> ((motor.getTemperature() * 1.8) + 32), null);
    }
}
