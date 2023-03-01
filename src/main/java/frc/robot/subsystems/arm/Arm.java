// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.Lidar;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.utilities.FancyArmFeedForward;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Positions;
import frc.robot.utilities.homing.HomeableCANSparkMax;
import frc.robot.utilities.homing.HomeableSubsystem;
import frc.robot.utilities.lists.Ports;

public class Arm extends SubsystemBase implements HomeableSubsystem, Loggable {
  
  public static final Transform3d ROBOT_TO_TURRET_BASE = new Transform3d(new Translation3d(-0.2413, 0, 0.18265), new Rotation3d());
  // public static final Transform3d ROBOT_TO_TURRET_BASE = new Transform3d(new Translation3d(), new Rotation3d());
  public static final double
    TURRET_P = 5E-5,
    TURRET_I = 1E-6,
    TURRET_D = 0,

    ARM_JOINT_1_P = 5E-5,
    ARM_JOINT_1_I = 1E-6,
    ARM_JOINT_1_D = 0,

    ARM_JOINT_2_P = 5E-5,
    ARM_JOINT_2_I = 1E-6,
    ARM_JOINT_2_D = 0,
    
    ARM_JOINT_3_P = 4E-5,
    ARM_JOINT_3_I = 2E-7,
    ARM_JOINT_3_D = 0,

    WRIST_P = 4E-5,
    WRIST_I = 2E-7,
    WRIST_D = 0,
    
    ARM_LINKAGE_0_LENGTH = 8 / 39.3701, // Length in meters
    ARM_LINKAGE_1_LENGTH = 31 / 39.3701, // Length in meters
    ARM_LINKAGE_2_LENGTH = 29 / 39.3701, // Length in meters
    ARM_LINKAGE_3_LENGTH = 18.125 / 39.3701, // Length in meters

    ARM_LINKAGE_1_CG_DISTANCE = 14 / 39.3701, // Distance from the pivot to the center of gravity in meters
    ARM_LINKAGE_2_CG_DISTANCE = 19 / 39.3701, // Distance from the pivot to the center of gravity in meters
    ARM_LINKAGE_3_CG_DISTANCE = 8 / 39.3701, // Distance from the pivot to the center of gravity in meters

    ARM_LINKAGE_1_MASS = 12 / 2.205, // Mass in kilograms
    ARM_LINKAGE_2_MASS = 8 / 2.205, // Mass in kilograms
    ARM_LINKAGE_3_MASS = 7 / 2.205, // Mass in kilograms
    
    KG_TO_NEWTONS = 9.80665;

    public static final float

    ARM_TURRET_FORWARD_SOFT_LIMIT = 61.75f,
    ARM_TURRET_REVERSE_SOFT_LIMIT = 1,
    ARM_JOINT_1_FORWARD_SOFT_LIMIT = 118,
    ARM_JOINT_1_REVERSE_SOFT_LIMIT = 1,
    ARM_JOINT_2_FORWARD_SOFT_LIMIT = 138,
    ARM_JOINT_2_REVERSE_SOFT_LIMIT = 1,
    ARM_JOINT_3_FORWARD_SOFT_LIMIT = -10,
    ARM_JOINT_3_REVERSE_SOFT_LIMIT = -175,
    ARM_WRIST_FORWARD_SOFT_LIMIT = -6,
    ARM_WRIST_REVERSE_SOFT_LIMIT = -100;

    public static final double

    TURRET_GEAR_RATIO_OVERALL = 27 * 3.09523809524, // Ratio Example a 9:1 would be 9
    TURRET_HOME_ANGLE = 2.394019735, // Angle in radians where 0 is straight forward and positive is counter clockwise.

    ARM_JOINT_1_LEADSCREW_HOME_LENGTH = 0.2635, // Length in meters
    ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH = 0.1016, // Length in meters
    ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH = 0.2270125, // Length in meters
    ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET = Math.toRadians(14.8) - 0.11472846, // Angle in radians
    ARM_JOINT_1_MOTOR_GEAR_RATIO = 9, // Ratio Example a 9:1 gear ratio would be 9
    ARM_JOINT_1_LEADSCREW_PITCH = 0.00635, // Length in meters. The distance the lead screw moves per revolution

    ARM_JOINT_2_LEADSCREW_HOME_LENGTH = 0.422275, // Length in meters
    ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH = 0.3360166, // Length in meters
    ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH = 0.0999998, // Length in meters
    ARM_JOINT_2_PIVOT_TO_MOTOR_VERTICAL_ANGLE_OFFSET = Math.toRadians(15.4) + 0.05184132679, // Angle in radians
    ARM_JOINT_2_MOTOR_GEAR_RATIO = 5, // Ratio Example a 9:1 gear ratio would be 9
    ARM_JOINT_2_LEADSCREW_PITCH = 0.00635, // Length in meters. The distance the lead screw moves per revolution

    ARM_JOINT_3_GEAR_RATIO_OVERALL = 350, // Ratio Example a 9:1 would be 9
    ARM_JOINT_3_HOME_ANGLE = -1.62301, // Angle in radians where 0 is straight forward and positive is counter clockwise.

    WRIST_GEAR_RATIO_OVERALL = (5*5*4) * (77/42), // Ratio Example a 9:1 would be 9
    WRIST_HOME_ANGLE = -2.96953297, // Angle in radians where 0 is straight forward and positive is counter clockwise.

    LIDAR_CLAMP_NEAR = 55,
    LIDAR_CLAMP_FAR = 60;
  
  private final CANSparkMax
    turretMotor = new CANSparkMax(Ports.Arm.TURRET, MotorType.kBrushless),
    joint1Motor = new CANSparkMax(Ports.Arm.JOINT_1, MotorType.kBrushless),
    joint2Motor = new CANSparkMax(Ports.Arm.JOINT_2, MotorType.kBrushless),
    joint3Motor = new CANSparkMax(Ports.Arm.JOINT_3, MotorType.kBrushless),
    wristMotor = new CANSparkMax(Ports.Arm.WRIST, MotorType.kBrushless);

  private final SparkMaxPIDController
    turretPIDController = turretMotor.getPIDController(),
    joint1PIDController = joint1Motor.getPIDController(),
    joint2PIDController = joint2Motor.getPIDController(),
    joint3PIDController = joint3Motor.getPIDController(),
    wristPIDController = wristMotor.getPIDController();

  public static FancyArmFeedForward
    joint1FF = new FancyArmFeedForward(0,0,0,(ARM_LINKAGE_1_MASS + ARM_LINKAGE_2_MASS + ARM_LINKAGE_3_MASS) * KG_TO_NEWTONS,2.6),
    joint2FF = new FancyArmFeedForward(0,0,0,(ARM_LINKAGE_2_MASS + ARM_LINKAGE_3_MASS) * KG_TO_NEWTONS,2.6),
    joint3FF = new FancyArmFeedForward(0,0,0,(ARM_LINKAGE_3_MASS) * KG_TO_NEWTONS,0.97);

  private final RelativeEncoder
    turretEncoder = turretMotor.getEncoder(),
    joint1Encoder = joint1Motor.getEncoder(),
    joint2Encoder = joint2Motor.getEncoder(),
    joint3Encoder = joint3Motor.getEncoder(),
    wirstEncoder = wristMotor.getEncoder();

  private ArmConfiguration currentConfiguration = new ArmConfiguration();
  private ArmConfiguration targetConfiguration = new ArmConfiguration();

    // Seperate boolean to store clamp state because it is slow to get the state of the solenoid.
  private final Solenoid clampSolenoid = new Solenoid(Ports.Other.PCM, PneumaticsModuleType.REVPH, Ports.Arm.CLAMP_SOLENOID);
  private boolean clampSolenoidState;

  private final Lidar lidar;

  /** 
   * Creates a new Arm.
   * The arm consists of a turret and Four joints.
   * The turret provides rotation arround the vertical axis.
   * Three of the joints pivot the arm up and down.
   * The fourth joint swivils a claw on the end of the arm.
   * The two final joints use servos and this provides for
   * better position control as PID is not needed.
   */
  public Arm(Lidar lidar) {

    this.lidar = lidar;

    turretPIDController.setP(TURRET_P, 0);
    turretPIDController.setI(TURRET_I, 0);
    turretPIDController.setD(TURRET_D, 0);
    turretPIDController.setFF(0.000156, 0);
    turretPIDController.setOutputRange(-1, 1, 0);
    turretPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    turretPIDController.setSmartMotionMaxAccel(1000, 0);
    turretPIDController.setSmartMotionMaxVelocity(6000, 0);

    joint1PIDController.setP(ARM_JOINT_1_P, 0);
    joint1PIDController.setI(ARM_JOINT_1_I, 0);
    joint1PIDController.setD(ARM_JOINT_1_D, 0);
    joint1PIDController.setFF(0.000156, 0);
    joint1PIDController.setOutputRange(-1, 1, 0);
    joint1PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    joint1PIDController.setSmartMotionMaxAccel(6000, 0);
    joint1PIDController.setSmartMotionMaxVelocity(6000, 0);

    joint2PIDController.setP(ARM_JOINT_2_P, 0);
    joint2PIDController.setI(ARM_JOINT_2_I, 0);
    joint2PIDController.setD(ARM_JOINT_2_D, 0);
    joint2PIDController.setFF(0.000156, 0);
    joint2PIDController.setOutputRange(-1, 1, 0);
    joint2PIDController.setOutputRange(-1, 1, 0);
    joint2PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    joint2PIDController.setSmartMotionMaxAccel(6000, 0);
    joint2PIDController.setSmartMotionMaxVelocity(6000, 0);

    joint3PIDController.setP(ARM_JOINT_3_P, 0);
    joint3PIDController.setI(ARM_JOINT_3_I, 0);
    joint3PIDController.setD(ARM_JOINT_3_D, 0);
    joint3PIDController.setFF(0.000156, 0);
    joint3PIDController.setOutputRange(-1, 1, 0);
    joint3PIDController.setOutputRange(-1, 1, 0);
    joint3PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    joint3PIDController.setSmartMotionMaxAccel(6000, 0);
    joint3PIDController.setSmartMotionMaxVelocity(12000, 0);

    wristPIDController.setP(WRIST_P,0);
    wristPIDController.setI(WRIST_I,0);
    wristPIDController.setD(WRIST_D,0);
    wristPIDController.setFF(0.000156, 0);
    wristPIDController.setOutputRange(-1, 1,0);
    wristPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    wristPIDController.setSmartMotionMaxAccel(6000, 0);
    wristPIDController.setSmartMotionMaxVelocity(12000, 0);

    turretMotor.clearFaults();
    joint1Motor.clearFaults();
    joint2Motor.clearFaults();
    joint3Motor.clearFaults();
    wristMotor.clearFaults();

    turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    joint1Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    joint2Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    joint3Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    turretMotor.setSoftLimit(SoftLimitDirection.kForward, ARM_TURRET_FORWARD_SOFT_LIMIT);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, ARM_TURRET_REVERSE_SOFT_LIMIT);

    joint1Motor.setSoftLimit(SoftLimitDirection.kForward, ARM_JOINT_1_FORWARD_SOFT_LIMIT);
    joint1Motor.setSoftLimit(SoftLimitDirection.kReverse, ARM_JOINT_1_REVERSE_SOFT_LIMIT);

    joint2Motor.setSoftLimit(SoftLimitDirection.kForward, ARM_JOINT_2_FORWARD_SOFT_LIMIT);
    joint2Motor.setSoftLimit(SoftLimitDirection.kReverse, ARM_JOINT_2_REVERSE_SOFT_LIMIT);

    joint3Motor.setSoftLimit(SoftLimitDirection.kForward, ARM_JOINT_3_FORWARD_SOFT_LIMIT);
    joint3Motor.setSoftLimit(SoftLimitDirection.kReverse, ARM_JOINT_3_REVERSE_SOFT_LIMIT);

    wristMotor.setSoftLimit(SoftLimitDirection.kForward, ARM_WRIST_FORWARD_SOFT_LIMIT);
    wristMotor.setSoftLimit(SoftLimitDirection.kReverse, ARM_WRIST_REVERSE_SOFT_LIMIT);

    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    joint1Motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    joint1Motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    joint2Motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    joint2Motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    joint3Motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    joint3Motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    clampSolenoidState = clampSolenoid.get();

    // sets refresh rate for various types of CAN data
    for (CANSparkMax motor : new CANSparkMax[] {turretMotor, joint1Motor, joint2Motor, joint3Motor, wristMotor}) {
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65533);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65531);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65529);
    }
  }

  /**
   * Sets the turret motor to the given voltage.
   * @param speed The speed to set the motor to.
   */
  public void setTurretMotorVoltage(double power) {
    power = Functions.clampDouble(power, 12, -12);
    turretMotor.setVoltage(power);
  }

  /**
   * Sets the joint 1 motor to the given voltage.
   * @param speed The speed to set the motor to.
   */
  public void setJoint1MotorVoltage(double power) {
    power = Functions.clampDouble(power, 12, -12);
    joint1Motor.setVoltage(power);
  }

  /**
   * Sets the joint 2 motor to the given voltage.
   * @param speed The speed to set the motor to.
   */
  public void setJoint2MotorVoltage(double power) {
    power = Functions.clampDouble(power, 12, -12);
    joint2Motor.setVoltage(power);
  }

  /**
   * Sets the joint 3 motor to the given voltage.
   * @param speed The speed to set the motor to.
   */
  public void setJoint3MotorVoltage(double power) {
    power = Functions.clampDouble(power, 12, -12);
    joint3Motor.setVoltage(power);
  }

  /**
   * Sets the wrist motor to the given voltage.
   * @param speed The speed to set the motor to.
   */
  public void setWristMotorVoltage(double power) {
    power = Functions.clampDouble(power, 12, -12);
    wristMotor.setVoltage(power);
  }

  /**
   * Sets the turret motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setTurretMotorRotations(double motorRotations) {
    turretPIDController.setReference(motorRotations, ControlType.kSmartMotion, 0);
  }

  /**
   * Sets the 1st joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setFirstJointMotorRotations(double motorRotations, double feedForward) {
    joint1PIDController.setReference(motorRotations, ControlType.kSmartMotion, 0, feedForward);
  }

  /**
   * Sets the 2nd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setSecondJointMotorRotations(double motorRotations, double feedForward) {
    joint2PIDController.setReference(motorRotations, ControlType.kSmartMotion, 0, feedForward);
  }

  /**
   * Sets the 3rd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setThirdJointMotorRotations(double motorRotations, double feedForward) {
    joint3PIDController.setReference(motorRotations, ControlType.kSmartMotion, 0, feedForward);
  }

  /**
   * Sets the wrist motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setWristMotorRotations(double motorRotations) {
    wristPIDController.setReference(motorRotations, ControlType.kSmartMotion, 0);
  }

  /**
   * Gets the turret's motor encoder position.
   * @return The turret motor's position in rotations.
   */
  public double getTurretEncoderPosition() {
    return turretEncoder.getPosition();
  }

  /**
   * Gets the 1st joint's motor encoder position.
   * @return The 1st joint motor's position in rotations.
   */
  public double getFirstJointEncoderPosition() {
    return joint1Encoder.getPosition();
  }

  /**
   * Gets the 2nd joint's motor encoder position.
   * @return The 2nd joint motor's position in rotations.
   */
  public double getSecondJointEncoderPosition() {
    return joint2Encoder.getPosition();
  }

  /**
   * Gets the 3rd joint's motor encoder position.
   * @return The 3rd joint motor's position in rotations.
   */
  public double getThirdJointEncoderPosition() {
    return joint3Encoder.getPosition();
  }

  /**
   * Gets the wrist's motor encoder position.
   * @return The wrist motor's position in rotations.
   */
  public double getWristEncoderPosition() {
    return wirstEncoder.getPosition();
  }

  /**
   * Returns the current configuration of the arm.
   * @return The current configuration of the arm.
   */
  public ArmConfiguration getCurrentArmConfiguration() {
    return currentConfiguration;
  }

  /**
   * Returns the target configuration of the arm.
   * @return The target configuration of the arm.
   * @return
   */
  public ArmConfiguration getTargetArmConfiguration() {
    return targetConfiguration;
  }

  /**
   * Sets the arm to a specific configuration.
   */
  public void setToConfiguration(ArmConfiguration configuration) {
    // System.out.println("Setting arm to configuration");
    if (!configuration.validConfig(getCurrentArmConfiguration())) return;
    // System.out.println("Valid configuration");

    setToConfigurationUnsafe(configuration);
  }

  public void setToConfigurationUnsafe(ArmConfiguration configuration) {

    targetConfiguration = configuration;
    double joint1ArbFF = joint1FF.calculate(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), configuration.getJoint1FFData());
    double joint2ArbFF = joint2FF.calculate(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), configuration.getJoint2FFData());
    double joint3ArbFF = joint3FF.calculate(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), configuration.getJoint3FFData());

    setTurretMotorRotations(configuration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS));
    setFirstJointMotorRotations(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint1ArbFF);
    setSecondJointMotorRotations(configuration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint2ArbFF);
    setThirdJointMotorRotations(configuration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint3ArbFF);
    setWristMotorRotations(configuration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS));
  }

  public boolean atConfiguration(ArmConfiguration configuration, double tolerance) {
    return currentConfiguration.equals(configuration, tolerance);
  }

  public boolean atTargetConfiguration(double tolerance) {
    return currentConfiguration.equals(targetConfiguration, tolerance);
  }

  /**
   * Actuates the clamp on the end of the arm
  */
  public void clamp() {
    clampSolenoid.set(false);
    clampSolenoidState = false;
  }

  /**
   * Releases the clamp on the end of the arm
   */
  public void unclamp() {
    clampSolenoid.set(true);
    clampSolenoidState = true;
  }

  public void toggleClamp() {
    if (clampSolenoidState) {
      clamp();
    } else {
      unclamp();
    }
  }

  /**
   * Gets the state of the clamp solenoid
   * @return The state of the clamp solenoid
   */
  public boolean getClampSolenoidState() {
    return clampSolenoidState;
  }

  

  @Override
  public HomeableCANSparkMax[] getHomeables() {
    return new HomeableCANSparkMax[] {
      new HomeableCANSparkMax(turretMotor, this, -0.1, 20.0, ARM_TURRET_FORWARD_SOFT_LIMIT, ARM_TURRET_REVERSE_SOFT_LIMIT, 0),
      new HomeableCANSparkMax(joint1Motor, this, -0.1, 30.0, ARM_JOINT_1_FORWARD_SOFT_LIMIT, ARM_JOINT_1_REVERSE_SOFT_LIMIT, 1),
      new HomeableCANSparkMax(joint2Motor, this, -0.1, 30.0, ARM_JOINT_2_FORWARD_SOFT_LIMIT, ARM_JOINT_2_REVERSE_SOFT_LIMIT, 1),
      new HomeableCANSparkMax(joint3Motor, this, 0.1, 10.0, ARM_JOINT_3_FORWARD_SOFT_LIMIT, ARM_JOINT_3_REVERSE_SOFT_LIMIT, 2),
      new HomeableCANSparkMax(wristMotor, this, 0.08, 4.5, ARM_WRIST_FORWARD_SOFT_LIMIT, ARM_WRIST_REVERSE_SOFT_LIMIT, 3)
    };
  }

  // encoder positions, grabber position, arm in sendable and loggable

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addStringProperty("armConfiguration", getCurrentArmConfiguration()::toString, null);
    builder.addStringProperty("grabberClamp", () -> clampSolenoidState ? "Open" : "Closed", null);
    builder.addDoubleProperty("turretEncoder", this::getTurretEncoderPosition, null);
    builder.addDoubleProperty("firstJointEncoder", this::getFirstJointEncoderPosition, null);
    builder.addDoubleProperty("secondJointEncoder", this::getSecondJointEncoderPosition, null);
    builder.addDoubleProperty("thirdJointEncoder", this::getThirdJointEncoderPosition, null);
    builder.addDoubleProperty("wristEncoder", this::getWristEncoderPosition, null);

    builder.addDoubleProperty("turretAngle", () -> this.getCurrentArmConfiguration().getTurretPosition(POSITION_TYPE.ANGLE), null);
    builder.addDoubleProperty("firstJointAngle", () -> this.getCurrentArmConfiguration().getFirstJointPosition(POSITION_TYPE.ANGLE), null);
    builder.addDoubleProperty("secondJointAngle", () -> this.getCurrentArmConfiguration().getSecondJointPosition(POSITION_TYPE.ANGLE), null);
    builder.addDoubleProperty("thirdJointAngle", () -> this.getCurrentArmConfiguration().getThirdJointPosition(POSITION_TYPE.ANGLE), null);
    builder.addDoubleProperty("wristAngle", () -> this.getCurrentArmConfiguration().getWristPosition(POSITION_TYPE.ANGLE), null);

    builder.addStringProperty("PosEstimateRS", () -> this.getCurrentArmConfiguration().getEndPosition().inRobotSpace().toString(), null);
    builder.addStringProperty("PosEstimateOS", () -> this.getCurrentArmConfiguration().getEndPosition().inOtherSpace(ROBOT_TO_TURRET_BASE).toString(), null);
    builder.addStringProperty("PosEstimateFS", () -> this.getCurrentArmConfiguration().getEndPosition().inFieldSpace().toString(), null);
  }

  @Override
  public String getLogName() {
    return "Arm";
  }

  @Override
  public HashMap<String, DoubleSupplier> getDoubleLogData() {
    HashMap<String, DoubleSupplier> out = new HashMap<String, DoubleSupplier>();
    out.put("Turret Encoder", this::getTurretEncoderPosition);
    out.put("First Joint Encoder", this::getFirstJointEncoderPosition);
    out.put("Second Joint Encoder", this::getSecondJointEncoderPosition);
    out.put("Third Joint Encoder", this::getThirdJointEncoderPosition);
    out.put("Wrist Encoder", this::getWristEncoderPosition);
    return out;
  }

  @Override
  public HashMap<String, Supplier<String>> getStringLogData() {
    HashMap<String, Supplier<String>> out = new HashMap<String, Supplier<String>>();
    out.put("Arm Configuration", getCurrentArmConfiguration()::toString);
    out.put("Grabber Clamp", () -> clampSolenoidState ? "Open" : "Closed");
    return out;
  }

  @Override
  public void periodic() {
    this.currentConfiguration = 
      new ArmConfiguration(
        getTurretEncoderPosition(),
        getFirstJointEncoderPosition(),
        getSecondJointEncoderPosition(),
        getThirdJointEncoderPosition(),
        getWristEncoderPosition(),
        POSITION_TYPE.ENCODER_ROTATIONS
      );
  }

  public void setTurretSoftLimit(boolean enable) {
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  }

  public void setFirstJointSoftLimit(boolean enable) {
    joint1Motor.enableSoftLimit(SoftLimitDirection.kForward, enable);
    joint1Motor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  }

  public void setSecondJointSoftLimit(boolean enable) {
    joint2Motor.enableSoftLimit(SoftLimitDirection.kForward, enable);
    joint2Motor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  }

  public void setThirdJointSoftLimit(boolean enable) {
    joint3Motor.enableSoftLimit(SoftLimitDirection.kForward, enable);
    joint3Motor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  }

  public void setWristSoftLimit(boolean enable) {
    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  }

  public void setAllSoftLimit(boolean enable) {
    setTurretSoftLimit(enable);
    setFirstJointSoftLimit(enable);
    setSecondJointSoftLimit(enable);
    setThirdJointSoftLimit(enable);
    setWristSoftLimit(enable);
  }

  public void stop() {
    setTurretMotorVoltage(0);
    setJoint1MotorVoltage(0);
    setJoint2MotorVoltage(0);
    setJoint3MotorVoltage(0);
    setWristMotorVoltage(0);
  }

  public boolean isWithinRange(Positions.Pose3d pose, double grabberAngle, double wristAngle) { 
    ArmConfiguration config = ArmConfiguration.fromEndPosition(pose, grabberAngle, wristAngle);
    if (!config.validConfig(getCurrentArmConfiguration())) {
      return false;
    }
    if (MovementMap.generatePathBetweenTwoPoints(getCurrentArmConfiguration().getEndPosition(), pose, MovementMap.getInstance().getMainMap()) == null) {
      return false;
    }
    return true;
  }

  public double getLidarDistance() {
    return lidar.getDistance();
  }
}
