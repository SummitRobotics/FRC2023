// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Ports;

public class Arm extends SubsystemBase {

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
    
    ARM_LINKAGE_0_LENGTH = 0, // Length in meters
    ARM_LINKAGE_1_LENGTH = 0, // Length in meters
    ARM_LINKAGE_2_LENGTH = 0, // Length in meters
    ARM_LINKAGE_3_LENGTH = 0, // Length in meters

    // Thease are the angles the joints are at when the encoders are at 0.
    HOME_TURRET_ANGLE = 0,       // Angle in radians where positive is counterclockwise
    HOME_ARM_JOINT_1_ANGLE = 0,  // Angle in radians where positive is counterclockwise
    HOME_ARM_JOINT_2_ANGLE = 0,  // Angle in radians where positive is counterclockwise
    HOME_ARM_JOINT_3_ANGLE = 0,  // Angle in radians where positive is counterclockwise
    HOME_WRIST_SWIVIL_ANGLE = 0, // Angle in radians where positive is counterclockwise

    TURRET_RADIANS_PER_ROTATION = 0,      // Prob Negitive beucause positive in the encoder is clockwise and positive in the arm is counterclockwise.
    ARM_JOINT_1_RADIANS_PER_ROTATION = 0,
    ARM_JOINT_2_RADIANS_PER_ROTATION = 0,
    ARM_JOINT_3_RADIANS = 0,
    WRIST_SWIVIL_RADIANS = 0;
  

  private final CANSparkMax
    turretMotor = new CANSparkMax(Ports.Arm.TURRET, MotorType.kBrushless),
    joint1Motor = new CANSparkMax(Ports.Arm.JOINT_1, MotorType.kBrushless),
    joint2Motor = new CANSparkMax(Ports.Arm.JOINT_2, MotorType.kBrushless);

  private final Servo
    joint3Servo = new Servo(Ports.Arm.JOINT_3),
    wristServo = new Servo(Ports.Arm.WRIST);

  private final SparkMaxPIDController
    turretPIDController = turretMotor.getPIDController(),
    joint1PIDController = joint1Motor.getPIDController(),
    joint2PIDController = joint2Motor.getPIDController();

  private final RelativeEncoder
    turretEncoder = turretMotor.getEncoder(),
    joint1Encoder = joint1Motor.getEncoder(),
    joint2Encoder = joint2Motor.getEncoder();

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

    turretMotor.clearFaults();
    joint1Motor.clearFaults();
    joint2Motor.clearFaults();

    turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    joint1Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    joint2Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
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
  public void setArmMainMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint1Motor.set(power);
  }

  /**
   * Sets the 2nd joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setArmSecondaryMotorPower(double power) {
    power = Functions.clampDouble(power, 1, -1);
    joint2Motor.set(power);
  }

  /**
   * Sets the turret motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setTurretPosition(double position) {
    turretPIDController.setReference(position, ControlType.kPosition);
  }

  /**
   * Sets the 1st joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setJoint1Position(double position) {
    joint1PIDController.setReference(position, ControlType.kPosition);
  }

  /**
   * Sets the 2nd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setJoint2Position(double position) {
    joint2PIDController.setReference(position, ControlType.kPosition);
  }

  /**
   * Sets the wrist servo to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setWristPosition(double position) {
    position = Functions.clampDouble(position, 1, 0);
    wristServo.set(position);
  }

  /**
   * Sets joint 3 servo to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setJoint3Position(double position) {
    position = Functions.clampDouble(position, 1, 0);
    joint3Servo.set(position);
  }

  /**
   * Gets the turret motor's position.
   * @return The turret motor's position in rotations.
   */
  public double getTurretPosition() {
    return turretEncoder.getPosition();
  }

  /**
   * Gets the 1st joint's motor position.
   * @return The joint's 1st motor position in rotations.
   */
  public double getJoint1Position() {
    return joint1Encoder.getPosition();
  }

  /**
   * Gets the 2st joint's motor position.
   * @return The joint's 2nd motor position in rotations.
   */
  public double getJoint2Position() {
    return joint2Encoder.getPosition();
  }

  /**
   * Gets the wrist servo position.
   * @return The wrist servo position where 0 is one extreme of servo and 1 is the other.
   */
  public double getWristPosition() {
    return wristServo.get();
  }

  /**
   * Gets the joint3 servo position.
   * @return The joint3 servo position where 0 is one extreme of servo and 1 is the other.
   */
  public double getJoint3Position() {
    return joint3Servo.get();
  }

  /**
   * Gets the Turrets Rotation as a Rotation3d.
   * @return The Turrets Rotation as a Rotation3d.
   */
  public Rotation3d getTurretRotation() {
    return new Rotation3d(0, 0, (getTurretPosition() * TURRET_RADIANS_PER_ROTATION) + HOME_TURRET_ANGLE);
  }

  /**
   * Gets the 1st joint's angle as a Rotation3d.
   * @return The 1st joint's angle as a Rotation3d.
   */
  public Rotation3d getJoint1Rotation() {
    return new Rotation3d(0, (getJoint1Position() * ARM_JOINT_1_RADIANS_PER_ROTATION) + HOME_ARM_JOINT_1_ANGLE, 0);
  }

  /**
   * Gets the 2nd joint's angle as a Rotation3d.
   * @return The 2nd joint's angle as a Rotation3d.
   */
  public Rotation3d getJoint2Rotation() {
    return new Rotation3d(0, (getJoint2Position() * ARM_JOINT_2_RADIANS_PER_ROTATION) + HOME_ARM_JOINT_2_ANGLE, 0);
  }

  /**
   * Gets the wrist's angle as a Rotation3d.
   * @return The wrist's angle as a Rotation3d.
   */
  public Rotation3d getJoint3Rotation() {
    return new Rotation3d(0, (getJoint2Position() * ARM_JOINT_3_RADIANS) + HOME_ARM_JOINT_2_ANGLE, 0);
  }

  /**
   * Gets the Turret's angle as a Rotation3d.
   * @return The Turret's angle as a Rotation3d.
   */
  public Rotation3d getWristRotation() {
    return new Rotation3d((getWristPosition() * WRIST_SWIVIL_RADIANS) + HOME_WRIST_SWIVIL_ANGLE, 0, 0);
  }

  /**
   * Gets the final position of the arm relitive to the base of the turret.
   * As well as its rotation. Used Forwards Kinematics.
   * @return The complete position of the arm.
   */
  public Transform3d calculateKinematics() {
    Translation3d linkage3 = new Translation3d(ARM_LINKAGE_3_LENGTH, getJoint3Rotation());
    Translation3d linkage2 = (new Translation3d(ARM_LINKAGE_2_LENGTH, getJoint2Rotation())).plus(linkage3.rotateBy(getJoint2Rotation()));
    Translation3d linkage1 = (new Translation3d(ARM_LINKAGE_1_LENGTH, getJoint1Rotation())).plus(linkage2.rotateBy(getJoint1Rotation()));
    Translation3d linkage0 = (new Translation3d(ARM_LINKAGE_0_LENGTH, getTurretRotation())).plus(linkage1.rotateBy(getTurretRotation()));
    
    Rotation3d wristRotation = getWristRotation().plus(getJoint3Rotation()).plus(getJoint2Rotation()).plus(getJoint1Rotation()).plus(getTurretRotation());
    return new Transform3d(linkage0, wristRotation);
  }
}
