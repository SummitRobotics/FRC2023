package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.AprilTagCameraWrapper;
import frc.robot.devices.AprilTagCameraWrapper.EstimatedRobotPoseWithSD;
import frc.robot.devices.LEDs.LEDCalls;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Testable;
import frc.robot.utilities.lists.Ports;

/**
 * Subsystem to control the drivetrain of the robot.
 */
public class Drivetrain extends SubsystemBase implements Testable {

    @AutoLog
    public static class DriveIOInputs {
        public double xPos = 0.0;
        public double yPos = 0.0;
        public double rotation = 0.0;
        public double leftEncoderPos = 0.0;
        public double rightEncoderPos = 0.0;
        public double leftEncoderVelocity = 0.0;
        public double rightEncoderVelocity = 0.0;
        public double leftDistanceAccumulator = 0.0;
        public double rightDistanceAccumulator = 0.0;
        public boolean shiftState = false;
    }

    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance;
    }

    public static Drivetrain init(AHRS gyro, Pose2d initPose2d) {
        if (instance == null) {
            instance = new Drivetrain(gyro, initPose2d);
        }
        return instance;
    }

    // TODO re tune/calculate all these
    public static final double 
        HIGH_GEAR_RATIO = 9.1,
        LOW_GEAR_RATIO = 19.65,
        WHEEL_RADIUS_IN_METERS = 0.075819,
        WHEEL_CIRCUMFERENCE_IN_METERS = (2 * WHEEL_RADIUS_IN_METERS) * Math.PI,
        MAX_OUTPUT_VOLTAGE = 11,
        DRIVE_WIDTH = -0.64135,
        SPLINE_MAX_VEL_MPS_HIGH = 3, // MAX:
        SPLINE_MAX_ACC_MPSSQ_HIGH = 3, // MAX :
        NO_FAULT_CODE = 0;

    public static final double
        HIGH_KS = 0.099848,    // Gains are in volts
        HIGH_KV = 2.4055,    // Gains are in M/s per vol
        HIGH_KA = 0.19389,    // Gains are in M/s^2 per volt
        HIGH_P = 0.05,
        HIGH_I = 0,
        HIGH_D = 0,
        HIGH_P_VEL = 2.1277E-7,
        HIGH_I_VEL = 0,
        HIGH_D_VEL = 0;


    // left motors
    private final CANSparkMax left =
        new CANSparkMax(Ports.Drivetrain.LEFT_3, MotorType.kBrushless);

    private final CANSparkMax leftMiddle =
        new CANSparkMax(Ports.Drivetrain.LEFT_2, MotorType.kBrushless);

    private final CANSparkMax leftBack =
        new CANSparkMax(Ports.Drivetrain.LEFT_1, MotorType.kBrushless);


    // right motors
    private final CANSparkMax right =
        new CANSparkMax(Ports.Drivetrain.RIGHT_3, MotorType.kBrushless);

    private final CANSparkMax rightMiddle =
        new CANSparkMax(Ports.Drivetrain.RIGHT_2, MotorType.kBrushless);

    private final CANSparkMax rightBack =
        new CANSparkMax(Ports.Drivetrain.RIGHT_1, MotorType.kBrushless);

    // pid controllers
    private final SparkMaxPIDController leftPID = left.getPIDController();
    private final SparkMaxPIDController rightPID = right.getPIDController();

    // encoders
    private final RelativeEncoder leftEncoder = left.getEncoder();
    private final RelativeEncoder rightEncoder = right.getEncoder();

    private final DifferentialDrivePoseEstimator poseEstimator;

    public final AHRS gyro;

    public static DifferentialDriveKinematics DriveKinematics =
        new DifferentialDriveKinematics(DRIVE_WIDTH);

    public static SimpleMotorFeedforward HighFeedForward =
        new SimpleMotorFeedforward(HIGH_KS, HIGH_KV, HIGH_KA);


    public static DifferentialDriveVoltageConstraint HighVoltageConstraint =
        new DifferentialDriveVoltageConstraint(HighFeedForward, DriveKinematics, MAX_OUTPUT_VOLTAGE);

    private final Solenoid shift;

    private boolean oldShift;

    //for making robot distance consistent across shifts
    private double leftDistanceAcum = 0;
    private double rightDistanceAcum = 0;

    private final Timer odometryTime = new Timer();

    private final Field2d f2d;
    private final ArrayList<AprilTagCameraWrapper> visionCameras = new ArrayList<>();
    
    /**
     * i am in PAIN wow this is BAD.
     *
     * @param gyro       odimetry is bads
     * @param initialPose the initial pose of the robot
     */
    private Drivetrain(AHRS gyro, Pose2d initialPose) {
        this.gyro = gyro;

        shift = new Solenoid(Ports.Other.PCM, PneumaticsModuleType.REVPH, Ports.Drivetrain.SHIFT_SOLENOID);

        odometryTime.reset();
        odometryTime.start();


        poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(-DRIVE_WIDTH), gyro.getRotation2d().unaryMinus(), 0, 0, initialPose);
        poseEstimator.setVisionMeasurementStdDevs((new Matrix<>(Nat.N3(), Nat.N1())).plus(1));

        f2d = new Field2d();
        f2d.setRobotPose(poseEstimator.getEstimatedPosition());

        // tells other two motors to follow the first
        leftMiddle.follow(left);
        leftBack.follow(left);

        rightMiddle.follow(right);
        rightBack.follow(right);

        // inverts right side
        left.setInverted(false);
        right.setInverted(true);

        zeroDistance();

        // pid for position
        leftPID.setP(HIGH_P, 1);
        leftPID.setI(HIGH_I, 1);
        leftPID.setD(HIGH_D, 1);
        leftPID.setOutputRange(-.5, .5, 1);

        rightPID.setP(HIGH_P, 1);
        rightPID.setI(HIGH_I, 1);
        rightPID.setD(HIGH_D, 1);
        rightPID.setOutputRange(-.5, .5, 1);
        
        // pid for velocity
        leftPID.setP(HIGH_P_VEL, 2);
        leftPID.setI(HIGH_I_VEL, 2);
        leftPID.setD(HIGH_D_VEL, 2);

        rightPID.setP(HIGH_P_VEL, 2);
        rightPID.setI(HIGH_I_VEL, 2);
        rightPID.setD(HIGH_D_VEL, 2);
        

        left.disableVoltageCompensation();
        right.disableVoltageCompensation();

        left.clearFaults();
        leftMiddle.clearFaults();
        leftBack.clearFaults();

        right.clearFaults();
        rightMiddle.clearFaults();
        rightBack.clearFaults();

        setClosedRampRate(0);
        setOpenRampRate(0);

        left.setSmartCurrentLimit(40);
        leftMiddle.setSmartCurrentLimit(40);
        leftBack.setSmartCurrentLimit(40);

        right.setSmartCurrentLimit(40);
        rightMiddle.setSmartCurrentLimit(40);
        rightBack.setSmartCurrentLimit(40);

        left.setIdleMode(IdleMode.kBrake);
        leftMiddle.setIdleMode(IdleMode.kBrake);
        leftBack.setIdleMode(IdleMode.kBrake);

        right.setIdleMode(IdleMode.kBrake);
        rightMiddle.setIdleMode(IdleMode.kBrake);
        rightBack.setIdleMode(IdleMode.kBrake);

        // We basically don't care about CAN data for follower motors.
        for (CANSparkMax motor : new CANSparkMax[] {leftMiddle, leftBack, rightMiddle, rightBack}) {
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65533);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65531);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65529);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65527);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65525);
        }

        // Speeding up frame 0 on the leader motors will increase the rate
        // at which followers are updated.
        left.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
        right.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    }

    /**
     * i am in PAIN wow this is BAD.
     *
     * @param gyro       odimetry is bad
     */
    public Drivetrain(AHRS gyro) {
        this(gyro, new Pose2d());
    }

    /**
     * Shifts the robot into high gear.
     */
    public void highGear() {
        synchronized (shift) {
            LEDCalls.LOW_GEAR.cancel();
            updateDistanceAcum();
            oldShift = false;
            shift.set(false);
        }
    }

    /**
     * Shifts the robot into low gear.
     */
    public void lowGear() {
        synchronized (shift) {
            LEDCalls.LOW_GEAR.activate();
            updateDistanceAcum();
            oldShift = true;
            shift.set(true);
        }
    }

    /**
     * Toggles the shift state.
     */
    public void toggleShift() {
        //System.out.println("shift call");
        if (oldShift) {
            highGear();
        } else {
            lowGear();
        }
    }

    /**
     * Updates the distace acumulator.
     */
    private void updateDistanceAcum() {
        leftDistanceAcum = getLeftDistance();
        rightDistanceAcum = getRightDistance();
        zeroEncoders();
    }

    /**
     * Gets the shift state.
     *
     * @return the shift state where true is low and false is high
     */
    public boolean getShift() {
        return oldShift;
    }

    /**
     * Sets the power of the left side of the drivetrain.
     *
     * @param power -1 - 1
     */
    public void setLeftMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        left.set(power);
    }

    /**
     * Sets the power of the right side of the drivetrain.
     *
     * @param power -1 - 1
     */
    public void setRightMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        right.set(power);
    }

    /**
     * Sets the power of both sides of the drivetrain.
     *
     * @param power The power, between -1 and 1
     */
    public synchronized void setBothMotorPower(double power) {
        power = Functions.clampDouble(power, 1.0, -1.0);
        left.set(power);
        right.set(power);
    }

    /**
     * Sets the voltage to the left motors.
     *
     * @param volts Amount of volts to send to left motors.
     */
    public void setLeftMotorVolts(double volts) {
        left.setVoltage(volts);
    }

    /**
     * Sets the voltage to the right motors.
     *
     * @param volts Amount of volts to send to the right motors.
     */
    public void setRightMotorVolts(double volts) {
        right.setVoltage(volts);
    }

    /**
     * Sets the voltage of the motors.
     *
     * @param left  the right motor voltage
     * @param right the left motor voltage
     */
    public void setMotorVolts(double left, double right) {
        //System.out.println(String.format("left is: %f, right is %f", left, right));
        setRightMotorVolts(right);
        setLeftMotorVolts(left);
    }

    /**
     * Sets the motor power based on desired meters per second.
     *
     * @param leftMS  the left motor MPS
     * @param rightMS the right motor MPS
     */
    public void setMotorTargetSpeed(double leftMS, double rightMS) {

        if (oldShift) {
            highGear();
        }

        // System.out.println(String.format("left is: %f, right is %f", leftMS, rightMS));

        double leftFeedForward = HighFeedForward.calculate(leftMS);
        double rightFeedForward = HighFeedForward.calculate(rightMS);

        leftPID.setReference(convertMPStoRPM(leftMS), ControlType.kVelocity, 2,  leftFeedForward);
        rightPID.setReference(convertMPStoRPM(rightMS), ControlType.kVelocity, 2, rightFeedForward);      
    }

    /**
     * maximum power PID can have. 
     *
     * @param power
     *
     */
    public void setPIDMaxPower(double power) {
        leftPID.setOutputRange(-power, power);
        rightPID.setOutputRange(-power, power);
    }

    /**
     * Converts robot meters per second into motor rotations per minute.
     *
     * @param input the MPS to convert
     * @return the corresponding RPM
     */
    public double convertMPStoRPM(double input) {
        double out = input / WHEEL_RADIUS_IN_METERS;
        out *= 60;
        out /= (2 * Math.PI);
        out *= HIGH_GEAR_RATIO;
        return out;
    }

    /**
     * Sets the target position of the left side of the drivetrain.
     *
     * @param position the target position in terms of motor rotations
     */
    public synchronized void setLeftMotorTarget(double position) {
        if (oldShift) {
            highGear();
        }
        leftPID.setReference(position, ControlType.kPosition, 1);
    }

    /**
     * Sets the target position of the right side of the drivetrain.
     *
     * @param position the target position in terms of motor rotations
     */
    public synchronized void setRightMotorTarget(double position) {
        if (oldShift) {
            highGear();
        }
        rightPID.setReference(position, ControlType.kPosition, 1);
    }

    /** drives to distance.
     *
     * @param position position to go to in rotations
     */
    public synchronized void setBothMotorTarget(double position) {
        if (oldShift) {
            highGear();
        }
        setLeftMotorTarget(position);
        setRightMotorTarget(position);
    }

    /**
     * Convert distance to encoder values.
     *
     * @param dist the distance in meters.
     * @return The converstion to encover values.
     */
    public double distToEncoder(double dist) {
        if (!getShift()) {
            return (dist / WHEEL_CIRCUMFERENCE_IN_METERS) * HIGH_GEAR_RATIO;
        } else {
            return (dist / WHEEL_CIRCUMFERENCE_IN_METERS) * LOW_GEAR_RATIO;
        }
    }

    /**
     * Convert encoder values to distance.
     *
     * @param encoder The encoder value
     * @return The conversion to distance in meters
     */
    public double encoderToDist(double encoder) {
        if (!getShift()) {
            return encoder * WHEEL_CIRCUMFERENCE_IN_METERS / HIGH_GEAR_RATIO;
        } else {
            return encoder * WHEEL_CIRCUMFERENCE_IN_METERS / LOW_GEAR_RATIO;
        }
    }

    /**
     * goes a distance.
     *
     * @param position distance to travel in meters
     */
    public void distanceToTravel(double position) {
        setBothMotorTarget(distToEncoder(position));
    }
    
    /**
     * The position you want the left side to register.
     * When it is in the position it is currently in
     *
     * @param position the position for the encoder to register in rotations
     */
    public synchronized void setLeftEncoder(double position) {
        leftEncoder.setPosition(position);
    }

    /**
     * The position you want the right side to register.
     * When it is in the position it is currently in
     *
     * @param position the position for the encoder to register in rotations
     */
    public synchronized void setRightEncoder(double position) {
        rightEncoder.setPosition(position);
        
    }
    
    /**
     * Zeros the raw encoder values (PROBABLY NOT WHAT YOU WANT).
     *
     * @apiNote This zeros the raw encoder values. This is provavly not what you want to do.
     *     Instead use zeroEncoders().
     */
    public synchronized void zeroEncoders() {
        setRightEncoder(0);
        setLeftEncoder(0);
    }

    /**
     * zeros the enocders and encoder acum. 
     *
     * @apiNote is probably what you want
     */
    public synchronized void zeroDistance() {
        leftDistanceAcum = 0;
        rightDistanceAcum = 0;
        zeroEncoders();
    }

    /**
     * Returns the current position of right side of the drivetrain.
     *
     * @return position of motor in rotations
     */
    public double getRightEncoderPosition() {
        synchronized (poseEstimator) {
            return rightEncoder.getPosition();
        }
    }

    /**
     * Returns the current position of right side of the drivetrain.
     *
     * @return position of motor in rotations
     */
    public double getLeftEncoderPosition() {
        synchronized (poseEstimator) {
            return leftEncoder.getPosition();
        }
    }

    public double getLeftRPM() {
        return leftEncoder.getVelocity();
    }

    public double getRightRPM() {
        return rightEncoder.getVelocity();
    }

    /**
     * Gets the total distance the left side has traveled since its last reset.
     *
     * @return the total distance in meters the side as travled sense the last reset
     */
    public double getLeftDistance() {
        synchronized (poseEstimator) {
            if (!getShift()) {
                return ((getLeftEncoderPosition() / HIGH_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS)
                    + leftDistanceAcum;
            } else {
                return ((getLeftEncoderPosition() / LOW_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS)
                    + leftDistanceAcum;
            }
        }
    }

    /**
     * Gets the total distance the left side has traveled since its last reset.
     *
     * @return the total distance in meters the side as travled sense the last reset
     */
    public double getRightDistance() {
        synchronized (poseEstimator) {
            if (!getShift()) {
                return ((getRightEncoderPosition() / HIGH_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS)
                    + rightDistanceAcum;
            } else {
                return ((getRightEncoderPosition() / LOW_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_IN_METERS)
                    + rightDistanceAcum;
            }
        }
    }

    /**
     * Gets the linear speed of the left side in m/s.
     *
     * @return the linear speed of the side in meters per second
     */
    public synchronized double getLeftSpeed() {
        if (!getShift()) {
            return convertRpmToMetersPerSecond((getLeftRPM() / HIGH_GEAR_RATIO));
        } else {
            return convertRpmToMetersPerSecond((getLeftRPM() / LOW_GEAR_RATIO));
        }
    }

    /**
     * Gets the linear speed of the right side in m/s.
     *
     * @return the linear speed of the side in meters per second
     */
    public synchronized double getRightSpeed() {
        if (!getShift()) {
            return convertRpmToMetersPerSecond((getRightRPM() / HIGH_GEAR_RATIO));
        } else {
            return convertRpmToMetersPerSecond((getRightRPM() / LOW_GEAR_RATIO));
        }
    }

    // could things be good for once please
    private double convertRpmToMetersPerSecond(double rpm) {
        return ((rpm / 60) * (2 * Math.PI)) * WHEEL_RADIUS_IN_METERS;
    }

    /**
     * Sets the rate at which the motors ramp up and down in open loop control mode.
     *
     * @param rate time in seconds to go from 0 to full power
     */
    public void setOpenRampRate(double rate) {
        left.setOpenLoopRampRate(rate);
        leftMiddle.setOpenLoopRampRate(rate);
        leftBack.setOpenLoopRampRate(rate);
        right.setOpenLoopRampRate(rate);
        rightMiddle.setOpenLoopRampRate(rate);
        rightBack.setOpenLoopRampRate(rate);
    }

    /**
     * Sets the rate at which the motors ramp up and down in closed loop control mode.
     *
     * @param rate time in seconds to go from 0 to full power
     */
    public void setClosedRampRate(double rate) {
        left.setClosedLoopRampRate(rate);
        leftMiddle.setClosedLoopRampRate(rate);
        leftBack.setClosedLoopRampRate(rate);
        right.setClosedLoopRampRate(rate);
        rightMiddle.setClosedLoopRampRate(rate);
        rightBack.setClosedLoopRampRate(rate);
    }

    public double getLeftMotorCurrent() {
        return (left.getOutputCurrent());
    }

    public double getRightMotorCurrent() {
        return (right.getOutputCurrent());
    }

    /**
     * Stops the motors.
     */
    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }

    public synchronized DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
    }

    /**
     * sets the curent pos and RESETS ENCODERS TO 0.
     *
     * @param pose the new pose
     */
    public void setPose(Pose2d pose) {
        synchronized (poseEstimator) {
            zeroDistance();
            poseEstimator.resetPosition(gyro.getRotation2d().unaryMinus(), 0, 0, pose);
        }
    }

    public Pose2d getPose() {
        synchronized (poseEstimator) {
            return poseEstimator.getEstimatedPosition();
        }
    }

    /**
     * gets the right pid values for the curent shift state.
     *
     * @return double array of p,i,d
     */
    public double[] getPid() {
        double[] out = { HIGH_P, HIGH_I, HIGH_D };
        return out;
    }

    /**
     * Gets the feed forward.
     *
     * @return The motors feed forward.
     */
    public SimpleMotorFeedforward getFeedForward() {
        return HighFeedForward;
    }
    
    /**
     * Gets the current voltage constant placed on the drivetrain.
     *
     * @return the current voltage constraint.
     */
    public DifferentialDriveVoltageConstraint getVoltageConstraint() {
        return HighVoltageConstraint;
    }

    /**
     * Returns the trajectory config for High Gear.
     */
    public TrajectoryConfig generateTrajectoryConfigHighGear() {
        return new TrajectoryConfig(SPLINE_MAX_VEL_MPS_HIGH, SPLINE_MAX_ACC_MPSSQ_HIGH)
            .setKinematics(DriveKinematics).addConstraint(HighVoltageConstraint).setReversed(false);
    }

    public Field2d getFieldWidget() {
        return f2d;
    }


    /**
     * Updates odometry.
     * It only updates at a rate of 500hz maximum.
     */
    public void updateOdometry(ArrayList<EstimatedRobotPoseWithSD> visionPoseEstimates) {
        synchronized (poseEstimator) {
            //prevemts unnessarly fast updates to the odemetry (2 ms)
            if (odometryTime.get() > 0.002) {
                poseEstimator.update(gyro.getRotation2d().unaryMinus(), getLeftDistance(), getRightDistance());
                odometryTime.reset();
            }

            Pose3d currentPose = new Pose3d(poseEstimator.getEstimatedPosition());
            for (EstimatedRobotPoseWithSD visionPoseEstimate : visionPoseEstimates) {
                if (currentPose.minus(visionPoseEstimate.estimatedRobotPose.estimatedPose).getTranslation().getNorm() < 100) {
                    poseEstimator.setVisionMeasurementStdDevs((new Matrix<>(Nat.N3(), Nat.N1())).plus(visionPoseEstimate.sd));
                    poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedRobotPose.estimatedPose.toPose2d(), visionPoseEstimate.estimatedRobotPose.timestampSeconds);
                }
            }
            f2d.setRobotPose(poseEstimator.getEstimatedPosition());
        }
    }

    public void updateOdometry() {
        ArrayList<EstimatedRobotPoseWithSD> visionPoseEstimates = new ArrayList<>();
        for (AprilTagCameraWrapper visionCamera : visionCameras) {
            Optional<EstimatedRobotPose> visionRobotPose = visionCamera.getEstimatedGlobalPose(getPose());
            visionRobotPose.ifPresent(poseEstimate -> {
                double distance = visionCamera.getTargetDistance();
                double SD = 0.15 * distance * distance;
                visionPoseEstimates.add(new EstimatedRobotPoseWithSD(poseEstimate, SD));
            });
        }
        updateOdometry(visionPoseEstimates);
    }

    public void addVisionCamera(AprilTagCameraWrapper visionPoseEstimator) {
        visionCameras.add(visionPoseEstimator);
    }

    public double getRotation() {
        return gyro.getRotation2d().unaryMinus().getDegrees();
    }

    /**
     * Method that runs once per scheduler cycle.
     */
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();

        // Update logs
        updateInputs(inputs);
        Logger.getInstance().processInputs("Drivetrain", inputs);
        Logger.getInstance().recordOutput("Odometry", getPose());
    }

    @Override
    public String getTestName() {
        return "Drivetrain";
    }

    @Override
    public ArrayList<CANSparkMax> getMotors() {
        ArrayList<CANSparkMax> result = new ArrayList<CANSparkMax>();
        result.add(left);
        result.add(right);
        return result;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("leftDistance", this::getLeftDistance, null);
        builder.addDoubleProperty("leftEncoder", this::getLeftEncoderPosition, null);
        builder.addDoubleProperty("leftRPM", this::getLeftRPM, null);
        builder.addDoubleProperty("leftSpeed", this::getLeftSpeed, null);

        builder.addDoubleProperty("rightDistance", this::getRightDistance, null);
        builder.addDoubleProperty("rightEncoder", this::getRightEncoderPosition, null);
        builder.addDoubleProperty("rightRPM", this::getRightRPM, null);
        builder.addDoubleProperty("rightSpeed", this::getRightSpeed, null);
        builder.addDoubleProperty("rotation", this::getRotation, null);
        //f2d.initSendable(builder);
        SmartDashboard.putData(f2d);
        builder.addDoubleProperty("x-pos", () -> this.getPose().getX(), null);
        builder.addDoubleProperty("y-pos", () -> this.getPose().getY(), null);
        builder.addDoubleProperty("rot-degrees", () -> this.getPose().getRotation().getDegrees(), null);

        // builder.addBooleanProperty("shifterStatus", this::getShift, null);
        //builder.addDoubleArrayProperty("pidValues", this::getPid, null);
    }

    public double turnSpeedToMotorPower(double turnSpeed) {
        double x = Functions.clampDouble(Math.abs(turnSpeed), 1, 0);
        double inner = 4.38442E8 - (4.06266E8 * x);
        double val = 1.37166 - (0.0000655072 * Math.sqrt(inner));
        return val * Math.signum(turnSpeed);
    }

    private void updateInputs(DriveIOInputs inputs) {
        inputs.xPos = getPose().getX();
        inputs.yPos = getPose().getY();
        inputs.rotation = getPose().getRotation().getDegrees();
        inputs.leftEncoderPos = leftEncoder.getPosition();
        inputs.rightEncoderPos = rightEncoder.getPosition();
        inputs.leftEncoderVelocity = leftEncoder.getVelocity();
        inputs.rightEncoderVelocity = rightEncoder.getVelocity();
        inputs.leftDistanceAccumulator = leftDistanceAcum;
        inputs.rightDistanceAccumulator = rightDistanceAcum;
        inputs.shiftState = getShift();
    }
}