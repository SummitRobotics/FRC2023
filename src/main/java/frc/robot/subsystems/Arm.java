// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.Map.Entry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Node;
import frc.robot.utilities.Region;
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
    
    ARM_JOINT_3_P = 0,
    ARM_JOINT_3_I = 0,
    ARM_JOINT_3_D = 0,
    ARM_JOINT_3_FF = 0,

    WRIST_P = 0,
    WRIST_I = 0,
    WRIST_D = 0,
    
    ARM_LINKAGE_0_LENGTH = 0, // Length in meters
    ARM_LINKAGE_1_LENGTH = 0, // Length in meters
    ARM_LINKAGE_2_LENGTH = 0, // Length in meters
    ARM_LINKAGE_3_LENGTH = 0; // Length in meters
  
  public static final Translation3d ROBOT_TO_TURRET_BASE = new Translation3d(0, 0, 0);

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

  private final Solenoid clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH,Ports.Arm.CLAMP_SOLENOID);
  // Seperate boolean to store clamp state because it is slow to get the state of the solenoid.
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
   * Sets the 3rd joint's motor to the given speed.
   * @param speed The speed to set the motor to.
   */
  public void setArmTertiaryMotorPower(double power) {
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
    turretPIDController.setReference(motorRotations, ControlType.kPosition);
  }

  /**
   * Sets the 1st joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setArmMainMotorRotations(double motorRotations) {
    joint1PIDController.setReference(motorRotations, ControlType.kPosition);
  }

  /**
   * Sets the 2nd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setArmSecondaryMotorRotations(double motorRotations) {
    joint2PIDController.setReference(motorRotations, ControlType.kPosition);
  }

  /**
   * Sets the 3rd joint's motor to the given position.
   * @param position The position to set the motor to in rotations.
   */
  public void setArmTertiaryMotorRotations(double motorRotations) {
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
  public double getArmMainEncoderPosition() {
    return joint1Encoder.getPosition();
  }

  /**
   * Gets the 2nd joint's motor encoder position.
   * @return The 2nd joint motor's position in rotations.
   */
  public double getArmSecondaryEncoderPosition() {
    return joint2Encoder.getPosition();
  }

  /**
   * Gets the 3rd joint's motor encoder position.
   * @return The 3rd joint motor's position in rotations.
   */
  public double getArmTertiaryEncoderPosition() {
    return joint3Encoder.getPosition();
  }

  /**
   * Gets the wrist's motor encoder position.
   * @return The wrist motor's position in rotations.
   */
  public double getWristEncoderPosition() {
    return wirstEncoder.getPosition();
  }

  // TODO: NEED TO COMPUTE THIS
  /**
   * Converts the given encoder position to an angle the turret is at
   * @return The angle the turret is at in radians with 0 being straight forward and positive being counterclockwise.
   */
  public double getTurretAngle() {
    return 0;
  }

  // TODO: NEED TO COMPUTE THIS
  /**
   * Converts the given encoder position to an angle the 1st joint is at
   * @return The angle the 1st joint is at in radians with 0 being straight and positive being counterclockwise.
   */
  public double getArmMainAngle() {
    return 0;
  }

  // TODO: NEED TO COMPUTE THIS
  /**
   * Converts the given encoder position to an angle the 2nd joint is at
   * @return The angle the 2nd joint is at in radians with 0 being straight and positive being counterclockwise.
   */
  public double getArmSecondaryAngle() {
    return 0;
  }

  // TODO: NEED TO COMPUTE THIS
  /**
   * Converts the given encoder position to an angle the 3rd joint is at
   * @return The angle the 3rd joint is at in radians with 0 being straight and positive being counterclockwise.
   */
  public double getArmTertiaryAngle() {
    return 0;
  }

  // TODO: NEED TO COMPUTE THIS
  /**
   * Converts the given encoder position to an angle the wrist is at
   * @return The angle the wrist is at in radians with 0 being straight and positive being counterclockwise.
   */
  public double getWristAngle() {
    return 0;
  }

  /**
   * Gets the Turrets Rotation as a Rotation3d.
   * @return The Turrets Rotation as a Rotation3d.
   */
  public Rotation3d getTurretRotation() {
    return new Rotation3d(0, 0, getTurretAngle());
  }

  /**
   * Gets the 1st Joint's Rotation as a Rotation3d.
   * @return The 1st Joint's Rotation as a Rotation3d.
   */
  public Rotation3d getArmMainRotation() {
    return new Rotation3d(0, getArmMainAngle(), 0);
  }

  /**
   * Gets the 2nd Joint's Rotation as a Rotation3d.
   * @return The 2nd Joint's Rotation as a Rotation3d.
   */
  public Rotation3d getArmSecondaryRotation() {
    return new Rotation3d(0, getArmSecondaryAngle(), 0);
  }

  /**
   * Gets the 3rd Joint's Rotation as a Rotation3d.
   * @return The 3rd Joint's Rotation as a Rotation3d.
   */
  public Rotation3d getArmTertiaryRotation() {
    return new Rotation3d(0, getArmTertiaryAngle(), 0);
  }

  /**
   * Gets the Wrist's Rotation as a Rotation3d.
   * @return The Wrist's Rotation as a Rotation3d.
   */
  public Rotation3d getWristRotation() {
    return new Rotation3d(getWristAngle(), 0, 0);
  }

  /**
   * Gets the final position of the arm relitive to the base of the turret.
   * As well as its rotation. Used Forwards Kinematics.
   * @return The complete position of the arm.
   */
  public Transform3d calculateKinematics() {
    Translation3d linkage3 = new Translation3d(ARM_LINKAGE_3_LENGTH, getArmTertiaryRotation());
    Translation3d linkage2 = (new Translation3d(ARM_LINKAGE_2_LENGTH, getArmSecondaryRotation())).plus(linkage3.rotateBy(getArmSecondaryRotation()));
    Translation3d linkage1 = (new Translation3d(ARM_LINKAGE_1_LENGTH, getArmMainRotation())).plus(linkage2.rotateBy(getArmMainRotation()));
    Translation3d linkage0 = (new Translation3d(ARM_LINKAGE_0_LENGTH, getTurretRotation())).plus(linkage1.rotateBy(getTurretRotation()));
    
    Rotation3d wristRotation = getWristRotation().plus(getArmTertiaryRotation()).plus(getArmSecondaryRotation()).plus(getArmMainRotation()).plus(getTurretRotation());
    return new Transform3d(linkage0, wristRotation);
  }

  /**
   * Sets the turret to a specific angle where 0 is straight forward and positive is counterclockwise.
   * @param angle The angle to set the turret to in radians.
   */

  public void setTurretAngle(double angle) {
    double rotations = angle * 0; // TODO CALUCLATE THIS
    setTurretMotorRotations(rotations);
  }

  /**
   * Sets the 1st joint to a specific angle where 0 is straight and positive is counterclockwise.
   * @param angle The angle to set the 1st joint to in radians.
   */
  public void setArmMainAngle(double angle) {
    double rotations = angle * 0; // TODO CALUCLATE THIS
    setArmMainMotorRotations(rotations);
  }

  /**
   * Sets the 2nd joint to a specific angle where 0 is straight and positive is counterclockwise.
   * @param angle The angle to set the 2nd joint to in radians.
   */
  public void setArmSecondaryAngle(double angle) {
    double rotations = angle * 0; // TODO CALUCLATE THIS
    setArmSecondaryMotorRotations(rotations);
  }

  /**
   * Sets the 3rd joint to a specific angle where 0 is straight and positive is counterclockwise.
   * @param angle The angle to set the 3rd joint to in radians.
   */
  public void setArmTertiaryAngle(double angle) {
    double rotations = angle * 0; // TODO CALUCLATE THIS
    setArmTertiaryMotorRotations(rotations);
  }

  /**
   * Sets the wrist to a specific angle where 0 is straight and positive is counterclockwise.
   * @param angle The angle to set the wrist to in radians.
   */
  public void setWristAngle(double angle) {
    double rotations = angle * 0; // TODO CALUCLATE THIS
    setWristMotorRotations(rotations);
  }

  /**
   * Sets the arm to grab a specific point in space.
   * @param grabberAngleRadians The angle of the grabber in radians compared to the ground. O is forward and PI/2 is straight down.
   * @param pointToGrab The point in space to grab relitive to the base of the turret. AKA the center of rotation for the turret.
   */
  public void setToPosition(double grabberAngleRadians, Translation3d pointToGrab) {
    // clamp grab angle
    grabberAngleRadians = Functions.clampDouble(grabberAngleRadians, Math.PI / 2, 0);
    // rotate turret to the same plane as pointToGrab
    double angleToPoint = Math.atan(pointToGrab.getY() / pointToGrab.getX());
    setTurretAngle(angleToPoint);

    // convert pointToGrab to 2d space
    Translation2d pointToGrab2d = pointToGrab
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

    // rotate joints to position
    setArmMainAngle(Math.atan(pointToGrab2d.getX() / pointToGrab2d.getY()) - alpha);
    setArmSecondaryAngle(Math.PI - gamma);
    setArmTertiaryAngle(Math.PI - beta - (Math.PI - grabberAngleRadians -
      (Math.PI / 2 - alpha - (Math.atan(pointToGrab2d.getX() / pointToGrab2d.getY()) - alpha))));
    setWristAngle(Math.PI / 2);
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

  public static class MovementMap {
    private static MovementMap instance;

    private final Set<Node<Region>> nodes = new HashSet<>();

    private MovementMap() {
          // TODO add nodes
    }

    public static MovementMap getInstance() {
      if (instance == null) {
        instance = new MovementMap();
      }
      return instance;
    }

    public List<Translation3d> generatePathBetweenTwoPoints(Translation3d start, Translation3d end) {
      Node<Region> startNode = null;
      for (Node<Region> node : nodes) {
        if (node.getData().contains(start)) {
          startNode = node;
          break;
        }
      }
      if (startNode == null) {
        throw new IllegalArgumentException("Start point is not in a region");
      }
      
      // Do Dijkstra's algorithm to find the shortest path to a region that contains the end point

      Set<NodeWrapper<Region>> settledNodes = new HashSet<>();
      Set<NodeWrapper<Region>> unsettledNodes = new HashSet<>();

      NodeWrapper<Region> startNodeWrapper = new NodeWrapper<>(startNode, 0);

      unsettledNodes.add(startNodeWrapper);

      while (unsettledNodes.size() != 0) {
        NodeWrapper<Region> currentNode = getLowestDistanceNode(unsettledNodes);
        if (currentNode.node.getData().contains(end)){
          List<Translation3d> path = new ArrayList<>();
          for (Node<Region> pathNode: currentNode.getShortestPath()) {
            path.add(pathNode.getData().getCenter());
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

    private class NodeWrapper<T> {
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
}
