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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.utilities.FancyArmFeedForward;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Positions;
import frc.robot.utilities.homing.HomeableCANSparkMax;
import frc.robot.utilities.homing.HomeableSubsystem;
import frc.robot.utilities.lists.Ports;

// TODO - override toString for ArmConfiguration
public class Arm extends SubsystemBase implements HomeableSubsystem, Loggable {
  
  public static final Transform3d ROBOT_TO_TURRET_BASE = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());
  public static final double
    TURRET_P = 0,
    TURRET_I = 0,
    TURRET_D = 0,

    ARM_JOINT_1_P = 0,
    ARM_JOINT_1_I = 0,
    ARM_JOINT_1_D = 0,

    ARM_JOINT_2_P = 0,
    ARM_JOINT_2_I = 0,
    ARM_JOINT_2_D = 0,
    
    ARM_JOINT_3_P = 0,
    ARM_JOINT_3_I = 0,
    ARM_JOINT_3_D = 0,

    WRIST_P = 0,
    WRIST_I = 0,
    WRIST_D = 0,
    
    ARM_LINKAGE_0_LENGTH = 8 / 39.37, // Length in meters
    ARM_LINKAGE_1_LENGTH = 31 / 39.37, // Length in meters
    ARM_LINKAGE_2_LENGTH = 29 / 39.37, // Length in meters
    ARM_LINKAGE_3_LENGTH = 18 / 39.37, // Length in meters

    ARM_LINKAGE_1_CG_DISTANCE = 14 / 39.37, // Distance from the pivot to the center of gravity in meters
    ARM_LINKAGE_2_CG_DISTANCE = 19 / 39.37, // Distance from the pivot to the center of gravity in meters
    ARM_LINKAGE_3_CG_DISTANCE = 8 / 39.37, // Distance from the pivot to the center of gravity in meters

    ARM_LINKAGE_1_MASS = 12 / 2.205, // Mass in kilograms
    ARM_LINKAGE_2_MASS = 8 / 2.205, // Mass in kilograms
    ARM_LINKAGE_3_MASS = 7 / 2.205, // Mass in kilograms
    
    KG_TO_NEWTONS = 9.80665;

    public static final double

    TURRET_GEAR_RATIO_OVERALL = 27 * 3.09523809524, // Ratio Example a 9:1 would be 9
    TURRET_HOME_ANGLE = 0, // Angle in radians where 0 is straight forward and positive is counter clockwise.

    ARM_JOINT_1_LEADSCREW_HOME_LENGTH = Units.inchesToMeters(16.5), // Length in meters
    ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH = 0.1019, // Length in meters
    ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH = 0.22606, // Length in meters
    ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET = Math.toRadians(14), // Angle in radians
    ARM_JOINT_1_MOTOR_GEAR_RATIO = 9, // Ratio Example a 9:1 gear ratio would be 9
    ARM_JOINT_1_LEADSCREW_PITCH = 0.00635, // Length in meters. The distance the lead screw moves per revolution

    ARM_JOINT_2_LEADSCREW_HOME_LENGTH = Units.inchesToMeters(4.875), // Length in meters
    ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH = 0.33655, // Length in meters
    ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH = 0.1001776, // Length in meters
    ARM_JOINT_2_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET = Math.toRadians(90 - 16.3), // Angle in radians
    ARM_JOINT_2_MOTOR_GEAR_RATIO = 9, // Ratio Example a 9:1 gear ratio would be 9
    ARM_JOINT_2_LEADSCREW_PITCH = 0.00635, // Length in meters. The distance the lead screw moves per revolution

    ARM_JOINT_3_GEAR_RATIO_OVERALL = 225, // Ratio Example a 9:1 would be 9
    ARM_JOINT_3_HOME_ANGLE = 0, // Angle in radians where 0 is straight forward and positive is counter clockwise.

    WRIST_GEAR_RATIO_OVERALL = 179.069767442, // Ratio Example a 9:1 would be 9
    WRIST_HOME_ANGLE = 0; // Angle in radians where 0 is straight forward and positive is counter clockwise.

  
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

  private FancyArmFeedForward
    joint1FF = new FancyArmFeedForward(0,0,0,(ARM_LINKAGE_1_MASS + ARM_LINKAGE_2_MASS + ARM_LINKAGE_3_MASS) * KG_TO_NEWTONS,2.6),
    joint2FF = new FancyArmFeedForward(0,0,0,(ARM_LINKAGE_2_MASS + ARM_LINKAGE_3_MASS) * KG_TO_NEWTONS,2.6),
    joint3FF = new FancyArmFeedForward(0,0,0,(ARM_LINKAGE_3_MASS) * KG_TO_NEWTONS,0.97);

  private final RelativeEncoder
    turretEncoder = turretMotor.getEncoder(),
    joint1Encoder = joint1Motor.getEncoder(),
    joint2Encoder = joint2Motor.getEncoder(),
    joint3Encoder = joint3Motor.getEncoder(),
    wirstEncoder = wristMotor.getEncoder();

    // Seperate boolean to store clamp state because it is slow to get the state of the solenoid.
  private final Solenoid clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH,Ports.Arm.CLAMP_SOLENOID);
  private boolean clampSolenoidState;

  /** 
   * Creates a new Arm.
   * The arm consists of a turret and Four joints.
   * The turret provides rotation arround the vertical axis.
   * Three of the joints pivot the arm up and down.
   * The fourth joint swivils a claw on the end of the arm.
   * The two final joints use servos and this provides for
   * better position control as PID is not needed.
   */
  public Arm() {
    turretPIDController.setP(TURRET_P, 0);
    turretPIDController.setI(TURRET_I, 0);
    turretPIDController.setD(TURRET_D, 0);

    joint1PIDController.setP(ARM_JOINT_1_P, 0);
    joint1PIDController.setI(ARM_JOINT_1_I, 0);
    joint1PIDController.setD(ARM_JOINT_1_D, 0);

    joint2PIDController.setP(ARM_JOINT_2_P, 0);
    joint2PIDController.setI(ARM_JOINT_2_I, 0);
    joint2PIDController.setD(ARM_JOINT_2_D, 0);

    joint3PIDController.setP(ARM_JOINT_3_P, 0);
    joint3PIDController.setI(ARM_JOINT_3_I, 0);
    joint3PIDController.setD(ARM_JOINT_3_D, 0);

    wristPIDController.setP(WRIST_P);
    wristPIDController.setI(WRIST_I);
    wristPIDController.setD(WRIST_D);

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

    clampSolenoidState = clampSolenoid.get();
  }

  /**
   * Sets the turret motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setTurretMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    turretMotor.set(power);
  }

  /**
   * Sets the 1st joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setFirstJointMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint1Motor.set(power);
  }

  /**
   * Sets the 2nd joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setSecondJointMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint2Motor.set(power);
  }

  /**
   * Sets the 3rd joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setThirdJointMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint3Motor.set(power);
  }

  /**
   * Sets the wrist motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setWristMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    wristMotor.set(power);
  }

  /**
   * Sets the turret motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setTurretMotorRotations(double motorRotations) {
    turretPIDController.setReference(motorRotations, ControlType.kPosition, 0);
  }

  /**
   * Sets the 1st joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setFirstJointMotorRotations(double motorRotations, double feedForward) {
    joint1PIDController.setReference(motorRotations, ControlType.kPosition, 0, feedForward);
  }

  /**
   * Sets the 2nd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setSecondJointMotorRotations(double motorRotations, double feedForward) {
    joint2PIDController.setReference(motorRotations, ControlType.kPosition, 0, feedForward);
  }

  /**
   * Sets the 3rd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setThirdJointMotorRotations(double motorRotations, double feedForward) {
    joint3PIDController.setReference(motorRotations, ControlType.kPosition, 0, feedForward);
  }

  /**
   * Sets the wrist motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setWristMotorRotations(double motorRotations) {
    wristPIDController.setReference(motorRotations, ControlType.kPosition, 0);
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
    return new ArmConfiguration(
      getTurretEncoderPosition(),
      getFirstJointEncoderPosition(),
      getSecondJointEncoderPosition(),
      getThirdJointEncoderPosition(),
      getWristEncoderPosition(),
      POSITION_TYPE.ENCODER_ROTATIONS
    );
  }

  /**
   * Sets the arm to a specific configuration.
   */
  public void setToConfiguration(ArmConfiguration configuration) {

    Pose3d linkage1CG = configuration.getLinkage1CG().inOtherSpace(ROBOT_TO_TURRET_BASE);
    Pose3d linkage2CG = configuration.getLinkage2CG().inOtherSpace(ROBOT_TO_TURRET_BASE);
    Pose3d linkage3CG = configuration.getLinkage3CG().inOtherSpace(ROBOT_TO_TURRET_BASE);

    Pose3d joint1Pose3d = configuration.getJoint1Pose().inOtherSpace(ROBOT_TO_TURRET_BASE);
    Pose3d joint2Pose3d = configuration.getJoint2Pose().inOtherSpace(ROBOT_TO_TURRET_BASE);
    Pose3d joint3Pose3d = configuration.getJoint3Pose().inOtherSpace(ROBOT_TO_TURRET_BASE);

    Positions.Pose3d pastJ2CG = ArmConfiguration.combineCG(
      Positions.Pose3d.fromRobotSpace(linkage2CG),
      Positions.Pose3d.fromRobotSpace(linkage3CG),
      ARM_LINKAGE_2_MASS,
      ARM_LINKAGE_3_MASS
    );

    Positions.Pose3d pastJ1CG = ArmConfiguration.combineCG(
      pastJ2CG,
      Positions.Pose3d.fromRobotSpace(linkage1CG),
      ARM_LINKAGE_3_MASS + ARM_LINKAGE_2_MASS,
      ARM_LINKAGE_1_MASS
    );

    double joint3CGDistance = joint3Pose3d.minus(linkage3CG).getTranslation().getNorm();
    double joint2CGDistance = joint2Pose3d.minus(pastJ2CG.inRobotSpace()).getTranslation().getNorm();
    double joint1CGDistance = joint1Pose3d.minus(pastJ1CG.inRobotSpace()).getTranslation().getNorm();

    double joint3CGAngle = Math.atan((joint3Pose3d.getZ() - linkage3CG.getZ()) / Math.sqrt(Math.pow(joint3Pose3d.getX() - linkage3CG.getX(), 2) + Math.pow(joint3Pose3d.getY() - linkage3CG.getY(), 2)));
    double joint2CGAngle = Math.atan((joint2Pose3d.getZ() - pastJ2CG.inRobotSpace().getZ()) / Math.sqrt(Math.pow(joint2Pose3d.getX() - pastJ2CG.inRobotSpace().getX(), 2) + Math.pow(joint2Pose3d.getY() - pastJ2CG.inRobotSpace().getY(), 2)));
    double joint1CGAngle = Math.atan((joint1Pose3d.getZ() - pastJ1CG.inRobotSpace().getZ()) / Math.sqrt(Math.pow(joint1Pose3d.getX() - pastJ1CG.inRobotSpace().getX(), 2) + Math.pow(joint1Pose3d.getY() - pastJ1CG.inRobotSpace().getY(), 2)));

    double joint1ArbFF = joint1FF.calculate(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint1CGDistance, joint1CGAngle, configuration.getFirstJointGearRatio());
    double joint2ArbFF = joint2FF.calculate(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint2CGDistance, joint2CGAngle, configuration.getSecondJointGearRatio());
    double joint3ArbFF = joint3FF.calculate(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint3CGDistance, joint3CGAngle, configuration.getThirdJointGearRatio());


    setTurretMotorRotations(configuration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS));
    setFirstJointMotorRotations(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint1ArbFF);
    setSecondJointMotorRotations(configuration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint2ArbFF);
    setThirdJointMotorRotations(configuration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), joint3ArbFF);
    setWristMotorRotations(configuration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS));
  }

  //TODO Make sure thease are good tollerances
  public boolean atConfiguration(ArmConfiguration configuration) {
    return (
      Functions.withinTolerance(getTurretEncoderPosition(), configuration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS), 0.1) &&
      Functions.withinTolerance(getFirstJointEncoderPosition(), configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), 0.1) &&
      Functions.withinTolerance(getSecondJointEncoderPosition(), configuration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), 0.1) &&
      Functions.withinTolerance(getThirdJointEncoderPosition(), configuration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS), 0.1) &&
      Functions.withinTolerance(getWristEncoderPosition(), configuration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS), 0.1)
    );
  }

  /**
   * Actuates the clamp on the end of the arm
  */
  public void clamp() {
    clampSolenoid.set(true);
    clampSolenoidState = true;
  }

  /**
   * Releases the clamp on the end of the arm
   */
  public void unclamp() {
    clampSolenoid.set(false);
    clampSolenoidState = false;
  }

  /**
   * Gets the state of the clamp solenoid
   * @return The state of the clamp solenoid
   */
  public boolean getClampSolenoidState() {
    return clampSolenoidState;
  }

  

  // TODO - check power and current values, check sequencing and what can be homed in parallel
  @Override
  public HomeableCANSparkMax[] getHomeables() {
    return new HomeableCANSparkMax[] {
      new HomeableCANSparkMax(turretMotor, this, 0.3, 15.0, 0),
      new HomeableCANSparkMax(joint1Motor, this, -0.3, 15.0, 1),
      new HomeableCANSparkMax(joint2Motor, this, 0.3, 15.0, 2),
      new HomeableCANSparkMax(joint3Motor, this, 0.3, 15.0, 3),
      new HomeableCANSparkMax(wristMotor, this, 0.3, 15.0, 4)
    };
  }

  // encoder positions, grabber position, arm in sendable and loggable

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("armConfiguration", getCurrentArmConfiguration()::toString, null);
    builder.addStringProperty("grabberClamp", () -> clampSolenoidState ? "Open" : "Closed", null);
    builder.addDoubleProperty("turretEncoder", this::getTurretEncoderPosition, null);
    builder.addDoubleProperty("firstJointEncoder", this::getFirstJointEncoderPosition, null);
    builder.addDoubleProperty("secondJointEncoder", this::getSecondJointEncoderPosition, null);
    builder.addDoubleProperty("thirdJointEncoder", this::getThirdJointEncoderPosition, null);
    builder.addDoubleProperty("wristEncoder", this::getWristEncoderPosition, null);
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
}
