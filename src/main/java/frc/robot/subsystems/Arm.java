// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.FancyArmFeedForward;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Node;
import frc.robot.utilities.Positions;
import frc.robot.utilities.Region;
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

  private final ChangeRateLimiter
    turretLimiter = new ChangeRateLimiter(0.1),
    joint1Limiter = new ChangeRateLimiter(0.1),
    joint2Limiter = new ChangeRateLimiter(0.1),
    joint3Limiter = new ChangeRateLimiter(0.1),
    wristLimiter = new ChangeRateLimiter(0.1);

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
    turretLimiter.resetOld(power);
  }

  /**
   * Sets the 1st joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setFirstJointMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint1Motor.set(power);
    joint1Limiter.resetOld(power);
  }

  /**
   * Sets the 2nd joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setSecondJointMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint2Motor.set(power);
    joint2Limiter.resetOld(power);
  }

  /**
   * Sets the 3rd joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setThirdJointMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint3Motor.set(power);
    joint3Limiter.resetOld(power);
  }

  /**
   * Sets the wrist motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setWristMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    wristMotor.set(power);
    wristLimiter.resetOld(power);
  }

  /**
   * Sets the turret motor to the given speed while being rate limited.
   * @param speed The speed to set the motor to.
   */
  public void setTurretMotorPowerRateLimited(double power) {
    power = Functions.clampDouble(power, 1, -1);
    turretMotor.set(turretLimiter.getRateLimitedValue(power));
  }

  /**
   * Sets the 1st joint's motor to the given speed while being rate limited.
   * @param speed The speed to set the motor to.
   */
  public void setFirstJointMotorPowerRateLimited(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint1Motor.set(joint1Limiter.getRateLimitedValue(power));
  }

  /**
   * Sets the 2nd joint's motor to the given speed while being rate limited.
   * @param speed The speed to set the motor to.
   */
  public void setSecondJointMotorPowerRateLimited(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint2Motor.set(joint2Limiter.getRateLimitedValue(power));
  }

  /**
   * Sets the 3rd joint's motor to the given speed while being rate limited.
   * @param speed The speed to set the motor to.
   */
  public void setThirdJointMotorPowerRateLimited(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint3Motor.set(joint3Limiter.getRateLimitedValue(power));
  }

  /**
   * Sets the wrist motor to the given speed while being rate limited.
   * @param speed The speed to set the motor to.
   */
  public void setWristMotorPowerRateLimited(double power) {
    power = Functions.clampDouble(power, 1, -1);
    wristMotor.set(wristLimiter.getRateLimitedValue(power));
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

    Positions.Pose3d pastJ2CG = ArmConfiguration.addTwoCG(
      Positions.Pose3d.fromRobotSpace(linkage2CG),
      Positions.Pose3d.fromRobotSpace(linkage3CG),
      ARM_LINKAGE_2_MASS,
      ARM_LINKAGE_3_MASS
    );

    Positions.Pose3d pastJ1CG = ArmConfiguration.addTwoCG(
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

  //Gets the angle the CG is from horrizontal from the pivot point for linkage 3.
  
  

  /**
   * Wrapper class for the arm position motors.
   */
  public static class ArmConfiguration {

    public enum POSITION_TYPE {
      ENCODER_ROTATIONS,
      ANGLE
    }

    private final double
      turretPositionRotations,
      firstJointPositionRotations,
      secondJointPositionRotations,
      thirdJointPositionRotations,
      wristPositionRotations;

    private final Positions.Pose3d endPose;

    private static double joint1AngleToEncoder(double angle) {
      double theta = (Math.PI / 2) + ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET - angle;
      double c = Math.sqrt(Math.pow(ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH, 2) - (2 * ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH * ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH) * Math.cos(theta));
      return -1 * (c - ARM_JOINT_1_LEADSCREW_HOME_LENGTH) * (ARM_JOINT_1_MOTOR_GEAR_RATIO / ARM_JOINT_1_LEADSCREW_PITCH);
    } 

    private static double joint2AngleToEncoder(double angle) {
      double theta = (Math.PI / 2) + ARM_JOINT_2_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET - angle;
      double c = Math.sqrt(Math.pow(ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH, 2) - (2 * ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH * ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH) * Math.cos(theta));
      return -1 * (c - ARM_JOINT_2_LEADSCREW_HOME_LENGTH) * (ARM_JOINT_2_MOTOR_GEAR_RATIO / ARM_JOINT_2_LEADSCREW_PITCH);
    }

    ArmConfiguration(
      double turretPosition,
      double firstJointPosition,
      double secondJointPosition,
      double thirdJointPosition,
      double wristPosition,
      POSITION_TYPE positionType
      ) {
        System.out.println(String.format("Turret: %f, First Joint: %f, Second Joint: %f, Third Joint: %f, Wrist: %f", turretPosition, firstJointPosition, secondJointPosition, thirdJointPosition, wristPosition));
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        this.turretPositionRotations = turretPosition;
        this.firstJointPositionRotations = firstJointPosition;
        this.secondJointPositionRotations = secondJointPosition;
        this.thirdJointPositionRotations = thirdJointPosition;
        this.wristPositionRotations = wristPosition;
      } else {
        this.turretPositionRotations = (TURRET_GEAR_RATIO_OVERALL/(-2 * Math.PI)) * (turretPosition - TURRET_HOME_ANGLE);
        this.firstJointPositionRotations = joint1AngleToEncoder(firstJointPosition);
        this.secondJointPositionRotations = joint2AngleToEncoder(secondJointPosition);
        this.thirdJointPositionRotations = (ARM_JOINT_3_GEAR_RATIO_OVERALL/(-2 * Math.PI)) * (thirdJointPosition - ARM_JOINT_3_HOME_ANGLE);
        this.wristPositionRotations = (WRIST_GEAR_RATIO_OVERALL/(-2 * Math.PI)) * (wristPosition - WRIST_HOME_ANGLE);
      }

      Translation2d linkage3 = new Translation2d(ARM_LINKAGE_3_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getThirdJointPosition(POSITION_TYPE.ANGLE)));
      Translation2d linkage2 = new Translation2d(ARM_LINKAGE_2_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE))).plus(linkage3.rotateBy(Rotation2d.fromRadians(-getSecondJointPosition(POSITION_TYPE.ANGLE))));
      Translation2d linkage1 = new Translation2d(ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
      Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,ARM_LINKAGE_0_LENGTH));

      this.endPose = Positions.Pose3d.fromOtherSpace(endPose, ROBOT_TO_TURRET_BASE);
    }

    public static ArmConfiguration fromEndPosition(Positions.Pose3d endPose, double grabberAngleRadians, double wristRotationRadians) {
      Translation3d endPosition = endPose.inOtherSpace(ROBOT_TO_TURRET_BASE).getTranslation();
      // clamp grab angle
      grabberAngleRadians = Functions.clampDouble(grabberAngleRadians, Math.PI / 2, 0);
      // rotate turret to the same plane as pointToGrab
      double angleToPoint = Math.atan(endPosition.getY() / endPosition.getX());

      // convert pointToGrab to 2d space
      Translation2d pointToGrab2d = endPosition
        .rotateBy(new Rotation3d(0,0,-angleToPoint))
        .rotateBy(new Rotation3d(-Math.PI / 2, 0, 0))
        .toTranslation2d()
        .plus(new Translation2d(0, -ARM_LINKAGE_0_LENGTH))
        .plus(new Translation2d(ARM_LINKAGE_3_LENGTH, new Rotation2d(Math.PI - grabberAngleRadians)));
      
      // solve the triangle with law of cosines
      double a = ARM_LINKAGE_1_LENGTH;
      double b = ARM_LINKAGE_2_LENGTH;
      double c = pointToGrab2d.getNorm();
      double alpha = Math.acos((a*a + b*b - c*c) / (2*a*b));
      double beta  = Math.acos((b*b + c*c - a*a) / (2*b*c));
      double gamma = Math.PI - alpha - beta;

      double j1 = (Math.PI / 2) - Math.atan(pointToGrab2d.getY() / pointToGrab2d.getX()) - gamma;
      double j2 = Math.PI - alpha;


      return new ArmConfiguration(
        angleToPoint,
        j1,
        j2,
        // Math.PI - beta - (Math.PI - grabberAngleRadians - (Math.PI / 2 - gamma - (Math.atan(pointToGrab2d.getX() / pointToGrab2d.getY()) - gamma))),
        (Math.PI / 2) + grabberAngleRadians - j1 - j2,
        wristRotationRadians,
        POSITION_TYPE.ANGLE
      );
    }

    public static double turretRotationsToAngle(double rotations) {
      return (rotations * (1/TURRET_GEAR_RATIO_OVERALL) * -2 * Math.PI) + TURRET_HOME_ANGLE;
    }

    public double getTurretPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return turretPositionRotations;
      }
      return turretRotationsToAngle(turretPositionRotations);
    }

    public static double joint1EncoderToAngle(double rotations) {
      double c = ARM_JOINT_1_LEADSCREW_HOME_LENGTH - (rotations * (1/ARM_JOINT_1_MOTOR_GEAR_RATIO) * ARM_JOINT_1_LEADSCREW_PITCH);
      double numerator = Math.pow(ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH, 2) +  Math.pow(ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH, 2) - Math.pow(c, 2);
      double denominator = 2 * ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH * ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH;
      return (Math.PI/2) - Math.acos(numerator / denominator) + ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET;
    }

    public double getFirstJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return firstJointPositionRotations;
      }
      return joint1EncoderToAngle(firstJointPositionRotations);
    }

    public static double joint2EncoderToAngle(double rotations) {
      double c = ARM_JOINT_2_LEADSCREW_HOME_LENGTH - (rotations * (1/ARM_JOINT_2_MOTOR_GEAR_RATIO) * ARM_JOINT_2_LEADSCREW_PITCH);
      double numerator = Math.pow(ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH, 2) - Math.pow(c, 2);
      double denominator = 2 * ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH * ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH;
      return (Math.PI/2) - Math.acos(numerator / denominator) + ARM_JOINT_2_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET;
    }

    public double getSecondJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return secondJointPositionRotations;
      }
      return joint2EncoderToAngle(secondJointPositionRotations);
    }

    public static double joint3EncoderToAngle(double rotations) {
      return (rotations * (1/ARM_JOINT_3_GEAR_RATIO_OVERALL) * -2 * Math.PI) + ARM_JOINT_3_HOME_ANGLE;
    }

    public double getThirdJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return thirdJointPositionRotations;
      }
      return joint3EncoderToAngle(thirdJointPositionRotations);
    }

    public static double wristEncoderToAngle(double rotations) {
      return (rotations * (1/WRIST_GEAR_RATIO_OVERALL) * -2 * Math.PI) + WRIST_HOME_ANGLE;
    }

    public double getWristPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return wristPositionRotations;
      }
      return wristEncoderToAngle(wristPositionRotations);
    }

    public Rotation3d getTurretRotation() {
      return new Rotation3d(0, 0, getTurretPosition(POSITION_TYPE.ANGLE));
    }

    public double getTurretGearRatio() {
      double dAngle = (turretRotationsToAngle(turretPositionRotations - 1) - turretRotationsToAngle(turretPositionRotations + 1));
      return ((2 * Math.PI)/dAngle) * 2;
    }

    public double getFirstJointGearRatio() {
      double dAngle = (joint1EncoderToAngle(firstJointPositionRotations - 1) - joint1EncoderToAngle(firstJointPositionRotations + 1));
      return ((2 * Math.PI)/dAngle) * 2;
    }

    public double getSecondJointGearRatio() {
      double dAngle = (joint2EncoderToAngle(secondJointPositionRotations - 1) - joint2EncoderToAngle(secondJointPositionRotations + 1));
      return ((2 * Math.PI)/dAngle) * 2;
    }

    public double getThirdJointGearRatio() {
      double dAngle = (joint3EncoderToAngle(thirdJointPositionRotations - 1) - joint3EncoderToAngle(thirdJointPositionRotations + 1));
      return ((2 * Math.PI)/dAngle) * 2;
    }

    public double getWristGearRatio() {
      double dAngle = (wristEncoderToAngle(wristPositionRotations - 1) - wristEncoderToAngle(wristPositionRotations + 1));
      return ((2 * Math.PI)/dAngle) * 2;
    }

    public Positions.Pose3d getEndPosition() {
      return endPose;
    }

    public Positions.Pose3d getJoint1Pose() {
      return Positions.Pose3d.fromOtherSpace(new Translation3d(0,0,ARM_LINKAGE_0_LENGTH), ROBOT_TO_TURRET_BASE);
    }

    public Positions.Pose3d getJoint2Pose() {
      Translation2d linkage1 = new Translation2d(ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE)));
      Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,ARM_LINKAGE_0_LENGTH));
      return Positions.Pose3d.fromOtherSpace(endPose, ROBOT_TO_TURRET_BASE);    }

    public Positions.Pose3d getJoint3Pose() {
      Translation2d linkage2 = new Translation2d(ARM_LINKAGE_2_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE)));
      Translation2d linkage1 = new Translation2d(ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
      Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,ARM_LINKAGE_0_LENGTH));
      return Positions.Pose3d.fromOtherSpace(endPose, ROBOT_TO_TURRET_BASE);
    }

    public Positions.Pose3d getLinkage1CG() {
      Translation2d linkage1 = new Translation2d(ARM_LINKAGE_1_CG_DISTANCE, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE)));
      Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,ARM_LINKAGE_0_LENGTH));
      return Positions.Pose3d.fromOtherSpace(endPose, ROBOT_TO_TURRET_BASE);
    }

    public Positions.Pose3d getLinkage2CG() {
      Translation2d linkage2 = new Translation2d(ARM_LINKAGE_2_CG_DISTANCE, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE)));
      Translation2d linkage1 = new Translation2d(ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
      Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,ARM_LINKAGE_0_LENGTH));
      return Positions.Pose3d.fromOtherSpace(endPose, ROBOT_TO_TURRET_BASE);
    }

    public Positions.Pose3d getLinkage3CG() {
      Translation2d linkage3 = new Translation2d(ARM_LINKAGE_3_CG_DISTANCE, Rotation2d.fromRadians((Math.PI / 2) - getThirdJointPosition(POSITION_TYPE.ANGLE)));
      Translation2d linkage2 = new Translation2d(ARM_LINKAGE_2_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE))).plus(linkage3.rotateBy(Rotation2d.fromRadians(-getSecondJointPosition(POSITION_TYPE.ANGLE))));
      Translation2d linkage1 = new Translation2d(ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
      Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,ARM_LINKAGE_0_LENGTH));
      return Positions.Pose3d.fromOtherSpace(endPose, ROBOT_TO_TURRET_BASE);
    }

    @Override
    public String toString() {
      return "Turret Encoder: " + turretPositionRotations
        + "\nFirst Joint Encoder: " + firstJointPositionRotations
        + "\nSecond Joint Encoder: " + secondJointPositionRotations
        + "\nThird Joint Encoder: " + thirdJointPositionRotations
        + "\nWrist Encoder: " + wristPositionRotations
        + "\nTurret Angle (rad): " + getTurretPosition(POSITION_TYPE.ANGLE)
        + "\nFirst Joint Angle (rad): " + getFirstJointPosition(POSITION_TYPE.ANGLE)
        + "\nSecond Joint Angle (rad): " + getSecondJointPosition(POSITION_TYPE.ANGLE)
        + "\nThird Joint Angle (rad): " + getThirdJointPosition(POSITION_TYPE.ANGLE)
        + "\nWrist Angle (rad): " + getWristPosition(POSITION_TYPE.ANGLE)
        + "\nTurret Angle (deg): " + Math.toDegrees(getTurretPosition(POSITION_TYPE.ANGLE))
        + "\nFirst Joint Angle (deg): " + Math.toDegrees(getFirstJointPosition(POSITION_TYPE.ANGLE))
        + "\nSecond Joint Angle (deg): " + Math.toDegrees(getSecondJointPosition(POSITION_TYPE.ANGLE))
        + "\nThird Joint Angle (deg): " + Math.toDegrees(getThirdJointPosition(POSITION_TYPE.ANGLE))
        + "\nWrist Angle (deg): " + Math.toDegrees(getWristPosition(POSITION_TYPE.ANGLE))
        + String.format("\nPosition (RS): (%.2f, %.2f, %.2f)", endPose.inRobotSpace().getX(), endPose.inRobotSpace().getY(), endPose.inRobotSpace().getZ())
        + String.format("\nJoint 1 Pos (RS): (%.2f, %.2f, %.2f)", getJoint1Pose().inRobotSpace().getX(), getJoint1Pose().inRobotSpace().getY(), getJoint1Pose().inRobotSpace().getZ())
        + String.format("\nJoint 2 Pos (RS): (%.2f, %.2f, %.2f)", getJoint2Pose().inRobotSpace().getX(), getJoint2Pose().inRobotSpace().getY(), getJoint2Pose().inRobotSpace().getZ())
        + String.format("\nJoint 3 Pos (RS): (%.2f, %.2f, %.2f)", getJoint3Pose().inRobotSpace().getX(), getJoint3Pose().inRobotSpace().getY(), getJoint3Pose().inRobotSpace().getZ())
        + String.format("\nLinkage 1 CG (RS): (%.2f, %.2f, %.2f)", getLinkage1CG().inRobotSpace().getX(), getLinkage1CG().inRobotSpace().getY(), getLinkage1CG().inRobotSpace().getZ())
        + String.format("\nLinkage 2 CG (RS): (%.2f, %.2f, %.2f)", getLinkage2CG().inRobotSpace().getX(), getLinkage2CG().inRobotSpace().getY(), getLinkage2CG().inRobotSpace().getZ())
        + String.format("\nLinkage 3 CG (RS): (%.2f, %.2f, %.2f)", getLinkage3CG().inRobotSpace().getX(), getLinkage3CG().inRobotSpace().getY(), getLinkage3CG().inRobotSpace().getZ());
    }

    public static Positions.Pose3d combineCG(Positions.Pose3d cg1, Positions.Pose3d cg2, double mass1, double mass2) {
      double totalMass = mass1 + mass2;
      double x = (cg1.inRobotSpace().getX() * mass1 + cg2.inRobotSpace().getX() * mass2) / totalMass;
      double y = (cg1.inRobotSpace().getY() * mass1 + cg2.inRobotSpace().getY() * mass2) / totalMass;
      double z = (cg1.inRobotSpace().getZ() * mass1 + cg2.inRobotSpace().getZ() * mass2) / totalMass;
      return Positions.Pose3d.fromRobotSpace(new Translation3d(x, y, z));
    }
  }

  /*
   * Class that is used to store the map of possible locations for the arm
   * to allow for movement without running into important parts of the robot.
   */
  public static class MovementMap {

    private final Set<Node<Region>> mainMap = new HashSet<>();

    public MovementMap() {
      // TODO ADD REGIONS TO MAP
      Node<Region> node1 = new Node<>(new Region(new Translation3d(0, 0, 0), new Translation3d(0, 0, 0)));
      Node<Region> node2 = new Node<>(new Region(new Translation3d(0, 0, 0), new Translation3d(0, 0, 0)));
      node1.addNeighboor(node2);
      node2.addNeighboor(node1);
      mainMap.add(node1);
      mainMap.add(node2);
    }

    public Set<Node<Region>> getMainMap() {
      return mainMap;
    }

    public static List<Positions.Pose3d> generatePathBetweenTwoPoints(Positions.Pose3d startPose, Positions.Pose3d endPose, Set<Node<Region>> map) {
      Translation3d start = startPose.inOtherSpace(ROBOT_TO_TURRET_BASE).getTranslation();
      Translation3d end = endPose.inOtherSpace(ROBOT_TO_TURRET_BASE).getTranslation();
      Node<Region> startNode = null;
      for (Node<Region> node : map) {
        if (node.getData().contains(start)) {
          startNode = node;
          break;
        }
      }
      if (startNode == null) {
        return null;
      }
      
      // Do Dijkstra's algorithm to find the shortest path to a region that contains the end point

      Set<NodeWrapper<Region>> settledNodes = new HashSet<>();
      Set<NodeWrapper<Region>> unsettledNodes = new HashSet<>();

      NodeWrapper<Region> startNodeWrapper = new NodeWrapper<>(startNode, 0);

      unsettledNodes.add(startNodeWrapper);

      while (unsettledNodes.size() != 0) {
        NodeWrapper<Region> currentNode = getLowestDistanceNode(unsettledNodes);
        if (currentNode.node.getData().contains(end)){
          List<Positions.Pose3d> path = new ArrayList<>();
          for (Node<Region> pathNode: currentNode.getShortestPath()) {
            path.add(Positions.Pose3d.fromOtherSpace(new Pose3d(pathNode.getData().getCenter(), new Rotation3d()), ROBOT_TO_TURRET_BASE));
          }
          return path;
        }
        unsettledNodes.remove(currentNode);
        for (Node<Region> neighboor: 
            currentNode.node.getNeighbors()) {
            double edgeWeight = currentNode.node.getData().distanceToOther(neighboor.getData());
            NodeWrapper<Region> neighboorNode = new NodeWrapper<>(neighboor, currentNode.distance + edgeWeight);
            if (!settledNodes.contains(neighboorNode)) {
              calculateMinimumDistance(neighboorNode, edgeWeight, currentNode);
              unsettledNodes.add(neighboorNode);
            }
          }
        settledNodes.add(currentNode);
      }
      return null;
    }

    private static NodeWrapper<Region> getLowestDistanceNode(Set<NodeWrapper<Region>> unsettledNodes) {
      NodeWrapper<Region> lowestDistanceNode = null;
      double lowestDistance = Double.MAX_VALUE;
      for (NodeWrapper<Region> node: unsettledNodes) {
            double nodeDistance = node.getDistance();
            if (nodeDistance < lowestDistance) {
                lowestDistance = nodeDistance;
                lowestDistanceNode = node;
            }
        }
        return lowestDistanceNode;
    }

    private static void calculateMinimumDistance(NodeWrapper<Region> evaluationNode, double edgeWeigh, NodeWrapper<Region> sourceNode) {
        double sourceDistance = sourceNode.getDistance();
        if (sourceDistance + edgeWeigh < evaluationNode.getDistance()) {
            evaluationNode.setDistance(sourceDistance + edgeWeigh);
            LinkedList<Node<Region>> shortestPath = new LinkedList<>(sourceNode.getShortestPath());
            shortestPath.add(sourceNode.getNode());
            evaluationNode.setShortestPath(shortestPath);
        }
    }

    private static class NodeWrapper<T> {
      private final Node<T> node;
      private double distance = Double.MAX_VALUE;
      private LinkedList<Node<T>> shortestPath = new LinkedList<>();

      public NodeWrapper(Node<T> node, double distance) {
        this.node = node;
        this.distance = distance;
      }

      public Node<T> getNode() {
        return node;
      }

      public double getDistance() {
        return distance;
      }

      public void setDistance(double distance) {
        this.distance = distance;
      }

      public LinkedList<Node<T>> getShortestPath() {
        return shortestPath;
      }

      public void setShortestPath(LinkedList<Node<T>> path) {
        this.shortestPath = path;
      }
    }
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
