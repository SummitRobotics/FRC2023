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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.utilities.ChangeRateLimiter;
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
    
    ARM_LINKAGE_0_LENGTH = 0, // Length in meters
    ARM_LINKAGE_1_LENGTH = 0, // Length in meters
    ARM_LINKAGE_2_LENGTH = 0, // Length in meters
    ARM_LINKAGE_3_LENGTH = 0, // Length in meters

    ARM_LINKAGE_1_WEIGHT = 0, // Weight in Newtons
    ARM_LINKAGE_2_WEIGHT = 0, // Weight in Newtons
    ARM_LINKAGE_3_WEIGHT = 0, // Weight in Newtons

    ARM_LINKAGE_1_CG_DISTANCE = 0, // Distance from the pivot point to the center of gravity in meters
    ARM_LINKAGE_2_CG_DISTANCE = 0, // Distance from the pivot point to the center of gravity in meters
    ARM_LINKAGE_3_CG_DISTANCE = 0; // Distance from the pivot point to the center of gravity in meters

    public static final double

    TURRET_GEAR_RATIO_OVERALL = 1, // Ratio Example a 9:1 would be 9
    TURRET_HOME_ANGLE = 0, // Angle in radians where 0 is straight forward and positive is counter clockwise.

    ARM_JOINT_1_LEADSCREW_HOME_LENGTH = 0, // Length in meters
    ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH = 0, // Length in meters
    ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH = 0, // Length in meters
    ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET = 0, // Angle in radians
    ARM_JOINT_1_MOTOR_GEAR_RATIO = 9, // Ratio Example a 9:1 gear ratio would be 9
    ARM_JOINT_1_LEADSCREW_PITCH = 0, // Length in meters. The distance the lead screw moves per revolution

    ARM_JOINT_2_LEADSCREW_HOME_LENGTH = 0, // Length in meters
    ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH = 0, // Length in meters
    ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH = 0, // Length in meters
    ARM_JOINT_2_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET = 0, // Angle in radians
    ARM_JOINT_2_MOTOR_GEAR_RATIO = 5, // Ratio Example a 9:1 gear ratio would be 9
    ARM_JOINT_2_LEADSCREW_PITCH = 0, // Length in meters. The distance the lead screw moves per revolution

    ARM_JOINT_3_GEAR_RATIO_OVERALL = 1, // Ratio Example a 9:1 would be 9
    ARM_JOINT_3_HOME_ANGLE = 0, // Angle in radians where 0 is straight forward and positive is counter clockwise.

    WRIST_GEAR_RATIO_OVERALL = 1, // Ratio Example a 9:1 would be 9
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
    turretPIDController.setReference(motorRotations, ControlType.kPosition);
  }

  /**
   * Sets the 1st joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setFirstJointMotorRotations(double motorRotations) {
    joint1PIDController.setReference(motorRotations, ControlType.kPosition);
  }

  /**
   * Sets the 2nd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setSecondJointMotorRotations(double motorRotations) {
    joint2PIDController.setReference(motorRotations, ControlType.kPosition);
  }

  /**
   * Sets the 3rd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setThirdJointMotorRotations(double motorRotations) {
    joint3PIDController.setReference(motorRotations, ControlType.kPosition);
  }

  /**
   * Sets the wrist motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setWristMotorRotations(double motorRotations) {
    wristPIDController.setReference(motorRotations, ControlType.kPosition);
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
    setTurretMotorRotations(configuration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS));
    setFirstJointMotorRotations(configuration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS));
    setSecondJointMotorRotations(configuration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS));
    setThirdJointMotorRotations(configuration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS));
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

    private final Positions.Pose3d position;

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

      Translation3d linkage3 = new Translation3d(ARM_LINKAGE_3_LENGTH, getThirdJointRotation());
      Translation3d linkage2 = (new Translation3d(ARM_LINKAGE_2_LENGTH, getSecondJointRotation())).plus(linkage3.rotateBy(getSecondJointRotation()));
      Translation3d linkage1 = (new Translation3d(ARM_LINKAGE_1_LENGTH, getFirstJointRotation())).plus(linkage2.rotateBy(getFirstJointRotation()));
      Translation3d linkage0 = (new Translation3d(ARM_LINKAGE_0_LENGTH, getTurretRotation())).plus(linkage1.rotateBy(getTurretRotation()));
      
      Rotation3d wristRotation = getWristRotation().plus(getThirdJointRotation()).plus(getSecondJointRotation()).plus(getFirstJointRotation()).plus(getTurretRotation());
      this.position = Positions.Pose3d.fromOtherSpace(new Pose3d(linkage0, wristRotation), ROBOT_TO_TURRET_BASE);
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
        .rotateBy(new Rotation3d(Math.PI / 2, 0, 0))
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

      return new ArmConfiguration(
        angleToPoint,
        Math.atan(pointToGrab2d.getX() / pointToGrab2d.getY()) - alpha,
        Math.PI - gamma,
        Math.PI - beta - (Math.PI - grabberAngleRadians - (Math.PI / 2 - alpha - (Math.atan(pointToGrab2d.getX() / pointToGrab2d.getY()) - alpha))),
        wristRotationRadians,
        POSITION_TYPE.ANGLE
      );
    }

    public double getTurretPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return turretPositionRotations;
      }
      return (turretPositionRotations * (1/TURRET_GEAR_RATIO_OVERALL) * -2 * Math.PI) + TURRET_HOME_ANGLE;
    }

    public double getFirstJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return firstJointPositionRotations;
      }
      double c = ARM_JOINT_1_LEADSCREW_HOME_LENGTH - (firstJointPositionRotations * (1/ARM_JOINT_1_MOTOR_GEAR_RATIO) * ARM_JOINT_1_LEADSCREW_PITCH);
      double numerator = Math.pow(ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH, 2) +  Math.pow(ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH, 2) - Math.pow(c, 2);
      double denominator = 2 * ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH * ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH;
      return (Math.PI/2) - Math.acos(numerator / denominator) + ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET;
    }

    public double getSecondJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return secondJointPositionRotations;
      }
      double c = ARM_JOINT_2_LEADSCREW_HOME_LENGTH - (secondJointPositionRotations * (1/ARM_JOINT_2_MOTOR_GEAR_RATIO) * ARM_JOINT_2_LEADSCREW_PITCH);
      double numerator = Math.pow(ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH, 2) - Math.pow(c, 2);
      double denominator = 2 * ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH * ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH;
      return (Math.PI/2) - Math.acos(numerator / denominator) + ARM_JOINT_2_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET;
    }

    public double getThirdJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return thirdJointPositionRotations;
      }
      return (thirdJointPositionRotations * (1/ARM_JOINT_3_GEAR_RATIO_OVERALL) * -2 * Math.PI) + ARM_JOINT_3_HOME_ANGLE;
    }

    public double getWristPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return wristPositionRotations;
      }
      return (wristPositionRotations * (1/WRIST_GEAR_RATIO_OVERALL) * -2 * Math.PI) + WRIST_HOME_ANGLE;
    }

    public Rotation3d getTurretRotation() {
      return new Rotation3d(0, 0, getTurretPosition(POSITION_TYPE.ANGLE));
    }

    public Rotation3d getFirstJointRotation() {
      return new Rotation3d(0, getFirstJointPosition(POSITION_TYPE.ANGLE), 0);
    }

    public Rotation3d getSecondJointRotation() {
      return new Rotation3d(0, getSecondJointPosition(POSITION_TYPE.ANGLE), 0);
    }

    public Rotation3d getThirdJointRotation() {
      return new Rotation3d(0, getThirdJointPosition(POSITION_TYPE.ANGLE), 0);
    }

    public Rotation3d getWristRotation() {
      return new Rotation3d(getWristPosition(POSITION_TYPE.ANGLE), 0, 0);
    }

    public Positions.Pose3d getEndPosition() {
      return position;
    }

    @Override
    public String toString() {
      return "Turret Encoder: " + turretPositionRotations
        + "\nFirst Joint Encoder: " + firstJointPositionRotations
        + "\nSecond Joint Encoder: " + secondJointPositionRotations
        + "\nThird Joint Encoder: " + thirdJointPositionRotations
        + "\nWrist Encoder: " + wristPositionRotations
        + "\nRobot Space X: " + position.inRobotSpace().getX()
        + "\nRobot Space Y: " + position.inRobotSpace().getY()
        + "\nRobot Space Z: " + position.inRobotSpace().getZ();
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
