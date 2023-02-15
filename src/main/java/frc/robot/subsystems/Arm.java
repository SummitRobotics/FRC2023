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
    ARM_JOINT_1_FF = 0,

    ARM_JOINT_2_P = 0,
    ARM_JOINT_2_I = 0,
    ARM_JOINT_2_D = 0,
    ARM_JOINT_2_FF = 0,
    
    ARM_JOINT_3_P = 0,
    ARM_JOINT_3_I = 0,
    ARM_JOINT_3_D = 0,
    ARM_JOINT_3_FF = 0,

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
    ARM_LINKAGE_3_MASS = 7 / 2.205; // Mass in kilograms
  
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
    turretPIDController.setP(TURRET_P);
    turretPIDController.setI(TURRET_I);
    turretPIDController.setD(TURRET_D);

    joint1PIDController.setP(ARM_JOINT_1_P);
    joint1PIDController.setI(ARM_JOINT_1_I);
    joint1PIDController.setD(ARM_JOINT_1_D);
    joint1PIDController.setFF(ARM_JOINT_1_FF);

    joint2PIDController.setP(ARM_JOINT_2_P);
    joint2PIDController.setI(ARM_JOINT_2_I);
    joint2PIDController.setD(ARM_JOINT_2_D);
    joint2PIDController.setFF(ARM_JOINT_2_FF);

    joint3PIDController.setP(ARM_JOINT_3_P);
    joint3PIDController.setI(ARM_JOINT_3_I);
    joint3PIDController.setD(ARM_JOINT_3_D);
    joint3PIDController.setFF(ARM_JOINT_3_FF);

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

    private final Positions.Pose3d 
      endPose,
      joint1Pose,
      joint2Pose,
      joint3Pose;

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
        this.turretPositionRotations = turretPosition * 0; // TODO CALCULATE THIS (ANGLE TO ROTATIONS)
        this.firstJointPositionRotations = firstJointPosition * 0; // TODO CALCULATE THIS (ANGLE TO ROTATIONS)
        this.secondJointPositionRotations = secondJointPosition * 0; // TODO CALCULATE THIS (ANGLE TO ROTATIONS)
        this.thirdJointPositionRotations = thirdJointPosition * 0; // TODO CALCULATE THIS (ANGLE TO ROTATIONS)
        this.wristPositionRotations = wristPosition * 0; // TODO CALCULATE THIS (ANGLE TO ROTATIONS)
      }

      Translation3d linkage3 = new Translation3d(ARM_LINKAGE_3_LENGTH, getThirdJointRotation());
      Translation3d linkage2 = (new Translation3d(ARM_LINKAGE_2_LENGTH, getSecondJointRotation())).plus(linkage3.rotateBy(getSecondJointRotation()));
      Translation3d linkage1 = (new Translation3d(ARM_LINKAGE_1_LENGTH, getFirstJointRotation())).plus(linkage2.rotateBy(getFirstJointRotation()));
      Translation3d linkage0 = (new Translation3d(ARM_LINKAGE_0_LENGTH, getTurretRotation())).plus(linkage1.rotateBy(getTurretRotation()));
      
      Rotation3d wristRotation = getWristRotation().plus(getThirdJointRotation()).plus(getSecondJointRotation()).plus(getFirstJointRotation()).plus(getTurretRotation());
      this.endPose = Positions.Pose3d.fromOtherSpace(new Pose3d(linkage0, wristRotation), ROBOT_TO_TURRET_BASE);

      linkage2 = new Translation3d(ARM_LINKAGE_2_LENGTH, getSecondJointRotation());
      linkage1 = (new Translation3d(ARM_LINKAGE_1_LENGTH, getFirstJointRotation())).plus(linkage2.rotateBy(getFirstJointRotation()));
      linkage0 = (new Translation3d(ARM_LINKAGE_0_LENGTH, getTurretRotation())).plus(linkage1.rotateBy(getTurretRotation()));

      this.joint3Pose = Positions.Pose3d.fromOtherSpace(linkage0, ROBOT_TO_TURRET_BASE);

      linkage1 = new Translation3d(ARM_LINKAGE_1_LENGTH, getFirstJointRotation());
      linkage0 = (new Translation3d(ARM_LINKAGE_0_LENGTH, getTurretRotation())).plus(linkage1.rotateBy(getTurretRotation()));

      this.joint2Pose = Positions.Pose3d.fromOtherSpace(linkage0, ROBOT_TO_TURRET_BASE);

      linkage0 = new Translation3d(ARM_LINKAGE_0_LENGTH, getTurretRotation());

      this.joint1Pose = Positions.Pose3d.fromOtherSpace(linkage0, ROBOT_TO_TURRET_BASE);
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
      return turretPositionRotations * 0; // TODO CALCULATE THIS (ROTATIONS TO ANGLE)
    }

    public double getFirstJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return firstJointPositionRotations;
      }
      return firstJointPositionRotations * 0; // TODO CALCULATE THIS (ROTATIONS TO ANGLE)
    }

    public double getSecondJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return secondJointPositionRotations;
      }
      return secondJointPositionRotations * 0; // TODO CALCULATE THIS (ROTATIONS TO ANGLE)
    }

    public double getThirdJointPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return thirdJointPositionRotations;
      }
      return thirdJointPositionRotations * 0; // TODO CALCULATE THIS (ROTATIONS TO ANGLE)
    }

    public double getWristPosition(POSITION_TYPE positionType) {
      if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
        return wristPositionRotations;
      }
      return wristPositionRotations * 0; // TODO CALCULATE THIS (ROTATIONS TO ANGLE)
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
      return endPose;
    }

    public Positions.Pose3d getJoint1Pose() {
      return joint1Pose;
    }

    public Positions.Pose3d getJoint2Pose() {
      return joint2Pose;
    }

    public Positions.Pose3d getJoint3Pose() {
      return joint3Pose;
    }

    public Positions.Pose3d getLinkage1CG() {
      Rotation3d rotation = getTurretRotation().plus(getFirstJointRotation());
      Pose3d pose = getJoint1Pose().inOtherSpace(ROBOT_TO_TURRET_BASE).plus(new Transform3d(new Translation3d(ARM_LINKAGE_1_CG_DISTANCE, rotation), new Rotation3d()));
      return Positions.Pose3d.fromOtherSpace(pose, ROBOT_TO_TURRET_BASE);
    }

    public Positions.Pose3d getLinkage2CG() {
      Rotation3d rotation = getTurretRotation().plus(getFirstJointRotation()).plus(getSecondJointRotation());
      Pose3d pose = getJoint2Pose().inOtherSpace(ROBOT_TO_TURRET_BASE).plus(new Transform3d(new Translation3d(ARM_LINKAGE_2_CG_DISTANCE, rotation), new Rotation3d()));
      return Positions.Pose3d.fromOtherSpace(pose, ROBOT_TO_TURRET_BASE);
    }

    public Positions.Pose3d getLinkage3CG() {
      Rotation3d rotation = getTurretRotation().plus(getFirstJointRotation()).plus(getSecondJointRotation()).plus(getThirdJointRotation());
      Pose3d pose = getJoint3Pose().inOtherSpace(ROBOT_TO_TURRET_BASE).plus(new Transform3d(new Translation3d(ARM_LINKAGE_3_CG_DISTANCE, rotation), new Rotation3d()));
      return Positions.Pose3d.fromOtherSpace(pose, ROBOT_TO_TURRET_BASE);
    }

    @Override
    public String toString() {
      return "Turret Encoder: " + turretPositionRotations
        + "\nFirst Joint Encoder: " + firstJointPositionRotations
        + "\nSecond Joint Encoder: " + secondJointPositionRotations
        + "\nThird Joint Encoder: " + thirdJointPositionRotations
        + "\nWrist Encoder: " + wristPositionRotations
        + String.format("Position (RS): (%.2f, %.2f, %.2f)", endPose.inRobotSpace().getX(), endPose.inRobotSpace().getY(), endPose.inRobotSpace().getZ())
        + String.format("Linkage 1 CG (RS): (%.2f, %.2f, %.2f)", getLinkage1CG().inRobotSpace().getX(), getLinkage1CG().inRobotSpace().getY(), getLinkage1CG().inRobotSpace().getZ())
        + String.format("Linkage 2 CG (RS): (%.2f, %.2f, %.2f)", getLinkage2CG().inRobotSpace().getX(), getLinkage2CG().inRobotSpace().getY(), getLinkage2CG().inRobotSpace().getZ())
        + String.format("Linkage 3 CG (RS): (%.2f, %.2f, %.2f)", getLinkage3CG().inRobotSpace().getX(), getLinkage3CG().inRobotSpace().getY(), getLinkage3CG().inRobotSpace().getZ());
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
