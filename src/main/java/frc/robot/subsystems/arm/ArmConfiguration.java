package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Positions;

public class ArmConfiguration {

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
    double theta = (Math.PI / 2) + Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET - angle;
    double c = Math.sqrt(Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH, 2) - (2 * Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH * Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH) * Math.cos(theta));
    return -1 * (c - Arm.ARM_JOINT_1_LEADSCREW_HOME_LENGTH) * (Arm.ARM_JOINT_1_MOTOR_GEAR_RATIO / Arm.ARM_JOINT_1_LEADSCREW_PITCH);
} 

private static double joint2AngleToEncoder(double angle) {
    double theta = (Math.PI / 2) + Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET - angle;
    double c = Math.sqrt(Math.pow(Arm.ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH, 2) - (2 * Arm.ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH * Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH) * Math.cos(theta));
    return -1 * (c - Arm.ARM_JOINT_2_LEADSCREW_HOME_LENGTH) * (Arm.ARM_JOINT_2_MOTOR_GEAR_RATIO / Arm.ARM_JOINT_2_LEADSCREW_PITCH);
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
    this.turretPositionRotations = (Arm.TURRET_GEAR_RATIO_OVERALL/(-2 * Math.PI)) * (turretPosition - Arm.TURRET_HOME_ANGLE);
    this.firstJointPositionRotations = joint1AngleToEncoder(firstJointPosition);
    this.secondJointPositionRotations = joint2AngleToEncoder(secondJointPosition);
    this.thirdJointPositionRotations = (Arm.ARM_JOINT_3_GEAR_RATIO_OVERALL/(-2 * Math.PI)) * (thirdJointPosition - Arm.ARM_JOINT_3_HOME_ANGLE);
    this.wristPositionRotations = (Arm.WRIST_GEAR_RATIO_OVERALL/(-2 * Math.PI)) * (wristPosition - Arm.WRIST_HOME_ANGLE);
    }

    Translation2d linkage3 = new Translation2d(Arm.ARM_LINKAGE_3_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getThirdJointPosition(POSITION_TYPE.ANGLE)));
    Translation2d linkage2 = new Translation2d(Arm.ARM_LINKAGE_2_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE))).plus(linkage3.rotateBy(Rotation2d.fromRadians(-getSecondJointPosition(POSITION_TYPE.ANGLE))));
    Translation2d linkage1 = new Translation2d(Arm.ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
    Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH));

    this.endPose = Positions.Pose3d.fromOtherSpace(endPose, Arm.ROBOT_TO_TURRET_BASE);
}

public static ArmConfiguration fromEndPosition(Positions.Pose3d endPose, double grabberAngleRadians, double wristRotationRadians) {
    Translation3d endPosition = endPose.inOtherSpace(Arm.ROBOT_TO_TURRET_BASE).getTranslation();
    // clamp grab angle
    grabberAngleRadians = Functions.clampDouble(grabberAngleRadians, Math.PI / 2, 0);
    // rotate turret to the same plane as pointToGrab
    double angleToPoint = Math.atan(endPosition.getY() / endPosition.getX());

    // convert pointToGrab to 2d space
    Translation2d pointToGrab2d = endPosition
    .rotateBy(new Rotation3d(0,0,-angleToPoint))
    .rotateBy(new Rotation3d(-Math.PI / 2, 0, 0))
    .toTranslation2d()
    .plus(new Translation2d(0, -Arm.ARM_LINKAGE_0_LENGTH))
    .plus(new Translation2d(Arm.ARM_LINKAGE_3_LENGTH, new Rotation2d(Math.PI - grabberAngleRadians)));
    
    // solve the triangle with law of cosines
    double a = Arm.ARM_LINKAGE_1_LENGTH;
    double b = Arm.ARM_LINKAGE_2_LENGTH;
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
    return (rotations * (1/Arm.TURRET_GEAR_RATIO_OVERALL) * -2 * Math.PI) + Arm.TURRET_HOME_ANGLE;
}

public double getTurretPosition(POSITION_TYPE positionType) {
    if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
    return turretPositionRotations;
    }
    return turretRotationsToAngle(turretPositionRotations);
}

public static double joint1EncoderToAngle(double rotations) {
    double c = Arm.ARM_JOINT_1_LEADSCREW_HOME_LENGTH - (rotations * (1/Arm.ARM_JOINT_1_MOTOR_GEAR_RATIO) * Arm.ARM_JOINT_1_LEADSCREW_PITCH);
    double numerator = Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH, 2) +  Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH, 2) - Math.pow(c, 2);
    double denominator = 2 * Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH * Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH;
    return (Math.PI/2) - Math.acos(numerator / denominator) + Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET;
}

public double getFirstJointPosition(POSITION_TYPE positionType) {
    if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
    return firstJointPositionRotations;
    }
    return joint1EncoderToAngle(firstJointPositionRotations);
}

public static double joint2EncoderToAngle(double rotations) {
    double c = Arm.ARM_JOINT_2_LEADSCREW_HOME_LENGTH - (rotations * (1/Arm.ARM_JOINT_2_MOTOR_GEAR_RATIO) * Arm.ARM_JOINT_2_LEADSCREW_PITCH);
    double numerator = Math.pow(Arm.ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH, 2) - Math.pow(c, 2);
    double denominator = 2 * Arm.ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH * Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH;
    return (Math.PI/2) - Math.acos(numerator / denominator) + Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET;
}

public double getSecondJointPosition(POSITION_TYPE positionType) {
    if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
    return secondJointPositionRotations;
    }
    return joint2EncoderToAngle(secondJointPositionRotations);
}

public static double joint3EncoderToAngle(double rotations) {
    return (rotations * (1/Arm.ARM_JOINT_3_GEAR_RATIO_OVERALL) * -2 * Math.PI) + Arm.ARM_JOINT_3_HOME_ANGLE;
}

public double getThirdJointPosition(POSITION_TYPE positionType) {
    if (positionType == POSITION_TYPE.ENCODER_ROTATIONS) {
    return thirdJointPositionRotations;
    }
    return joint3EncoderToAngle(thirdJointPositionRotations);
}

public static double wristEncoderToAngle(double rotations) {
    return (rotations * (1/Arm.WRIST_GEAR_RATIO_OVERALL) * -2 * Math.PI) + Arm.WRIST_HOME_ANGLE;
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
    return Positions.Pose3d.fromOtherSpace(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH), Arm.ROBOT_TO_TURRET_BASE);
}

public Positions.Pose3d getJoint2Pose() {
    Translation2d linkage1 = new Translation2d(Arm.ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE)));
    Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH));
    return Positions.Pose3d.fromOtherSpace(endPose, Arm.ROBOT_TO_TURRET_BASE);    }

public Positions.Pose3d getJoint3Pose() {
    Translation2d linkage2 = new Translation2d(Arm.ARM_LINKAGE_2_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE)));
    Translation2d linkage1 = new Translation2d(Arm.ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
    Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH));
    return Positions.Pose3d.fromOtherSpace(endPose, Arm.ROBOT_TO_TURRET_BASE);
}

public Positions.Pose3d getLinkage1CG() {
    Translation2d linkage1 = new Translation2d(Arm.ARM_LINKAGE_1_CG_DISTANCE, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE)));
    Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH));
    return Positions.Pose3d.fromOtherSpace(endPose, Arm.ROBOT_TO_TURRET_BASE);
}

public Positions.Pose3d getLinkage2CG() {
    Translation2d linkage2 = new Translation2d(Arm.ARM_LINKAGE_2_CG_DISTANCE, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE)));
    Translation2d linkage1 = new Translation2d(Arm.ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
    Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH));
    return Positions.Pose3d.fromOtherSpace(endPose, Arm.ROBOT_TO_TURRET_BASE);
}

public Positions.Pose3d getLinkage3CG() {
    Translation2d linkage3 = new Translation2d(Arm.ARM_LINKAGE_3_CG_DISTANCE, Rotation2d.fromRadians((Math.PI / 2) - getThirdJointPosition(POSITION_TYPE.ANGLE)));
    Translation2d linkage2 = new Translation2d(Arm.ARM_LINKAGE_2_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE))).plus(linkage3.rotateBy(Rotation2d.fromRadians(-getSecondJointPosition(POSITION_TYPE.ANGLE))));
    Translation2d linkage1 = new Translation2d(Arm.ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
    Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH));
    return Positions.Pose3d.fromOtherSpace(endPose, Arm.ROBOT_TO_TURRET_BASE);
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