package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utilities.Functions;
import frc.robot.utilities.Positions;
import frc.robot.utilities.FancyArmFeedForward.FFData;

public class ArmConfiguration {

public static final double VALID_POS_OFFSET = 1;

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

public static double joint1AngleToEncoder(double angle) {
    // System.out.println("Angle: " + Math.toDegrees(angle));
    double theta = (Math.PI / 2) - Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET - angle;
    // System.out.println("Theta: " + Math.toDegrees(theta));
    double c = Math.sqrt(Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH, 2) - (2 * Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH * Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH) * Math.cos(theta));
    // System.out.println("C: " + c);
    return (Arm.ARM_JOINT_1_LEADSCREW_HOME_LENGTH - c) * (Arm.ARM_JOINT_1_MOTOR_GEAR_RATIO / Arm.ARM_JOINT_1_LEADSCREW_PITCH);
} 

public static double joint2AngleToEncoder(double angle) {
    double theta = angle - Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_VERTICAL_ANGLE_OFFSET;
    double c = Math.sqrt(Math.pow(Arm.ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH, 2) + Math.pow(Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH, 2) - (2 * Arm.ARM_JOINT_2_PIVOT_TO_LEADSCREW_LENGTH * Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_LENGTH) * Math.cos(theta));
    return -1 * (c - Arm.ARM_JOINT_2_LEADSCREW_HOME_LENGTH) * (Arm.ARM_JOINT_2_MOTOR_GEAR_RATIO / Arm.ARM_JOINT_2_LEADSCREW_PITCH);
}

public ArmConfiguration(
    double turretPosition,
    double firstJointPosition,
    double secondJointPosition,
    double thirdJointPosition,
    double wristPosition,
    POSITION_TYPE positionType
    ) {
        // System.out.println(String.format("Turret: %.2f One: %.2f Two: %.2f Three: %.2f", turretPosition, firstJointPosition, secondJointPosition, thirdJointPosition));
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
}

ArmConfiguration() {
    this(0, 0, 0, 0, 0, POSITION_TYPE.ENCODER_ROTATIONS);
}

public static ArmConfiguration fromEndPosition(Positions.Pose3d endPose, double grabberAngleRadians, double wristRotationRadians) {
    Translation3d endPosition = endPose.inOtherSpace(Arm.ROBOT_TO_TURRET_BASE).getTranslation();    // clamp grab angle
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

    System.out.println(String.format("Turret: %.2f One: %.2f Two: %.2f Three: %.2f", angleToPoint, j1, j2,  (Math.PI / 2) + grabberAngleRadians - j1 - j2));

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
    // System.out.println(rotations);
    double c = (Arm.ARM_JOINT_1_LEADSCREW_HOME_LENGTH - (rotations * (1/Arm.ARM_JOINT_1_MOTOR_GEAR_RATIO) * Arm.ARM_JOINT_1_LEADSCREW_PITCH));
    // System.out.println(c);
    double numerator = Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH, 2) +  Math.pow(Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH, 2) - Math.pow(c, 2);
    double denominator = 2 * Arm.ARM_JOINT_1_PIVOT_TO_LEADSCREW_LENGTH * Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_LENGTH;
    // System.out.println(Math.toDegrees(Math.acos(numerator / denominator)));
    // System.out.println(Math.toDegrees((Math.PI/2) - Math.acos(numerator / denominator) + Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET));
    return (Math.PI/2) - Math.acos(numerator / denominator) - Arm.ARM_JOINT_1_PIVOT_TO_MOTOR_HORIZONTAL_ANGLE_OFFSET;
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
    return Math.acos(numerator / denominator) + Arm.ARM_JOINT_2_PIVOT_TO_MOTOR_VERTICAL_ANGLE_OFFSET;
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
    Translation2d linkage3 = new Translation2d(Arm.ARM_LINKAGE_3_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getThirdJointPosition(POSITION_TYPE.ANGLE)));
    Translation2d linkage2 = new Translation2d(Arm.ARM_LINKAGE_2_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getSecondJointPosition(POSITION_TYPE.ANGLE))).plus(linkage3.rotateBy(Rotation2d.fromRadians(-getSecondJointPosition(POSITION_TYPE.ANGLE))));
    Translation2d linkage1 = new Translation2d(Arm.ARM_LINKAGE_1_LENGTH, Rotation2d.fromRadians((Math.PI / 2) - getFirstJointPosition(POSITION_TYPE.ANGLE))).plus(linkage2.rotateBy(Rotation2d.fromRadians(-getFirstJointPosition(POSITION_TYPE.ANGLE))));
    Translation3d endPose = new Translation3d(linkage1.getX(), 0, linkage1.getY()).rotateBy(getTurretRotation()).plus(new Translation3d(0,0,Arm.ARM_LINKAGE_0_LENGTH));

    Rotation3d endRotation = new Rotation3d(0,(Math.PI / 2),0).plus(getTurretRotation()).plus(new Rotation3d(0,-getFirstJointPosition(POSITION_TYPE.ANGLE),0)).plus(new Rotation3d(0,-getSecondJointPosition(POSITION_TYPE.ANGLE),0)).plus(new Rotation3d(0,-getThirdJointPosition(POSITION_TYPE.ANGLE),0)).plus(new Rotation3d(getWristPosition(POSITION_TYPE.ANGLE),0,0));

    return Positions.Pose3d.fromOtherSpace(new Pose3d(endPose, endRotation), Arm.ROBOT_TO_TURRET_BASE);
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

public FFData getJoint1FFData() {
    Pose3d linkage1CG = getLinkage1CG().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);
    Pose3d linkage2CG = getLinkage2CG().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);
    Pose3d linkage3CG = getLinkage3CG().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);

    Pose3d joint1Pose3d = getJoint1Pose().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);

    Positions.Pose3d pastJ2CG = ArmConfiguration.combineCG(
      Positions.Pose3d.fromRobotSpace(linkage2CG),
      Positions.Pose3d.fromRobotSpace(linkage3CG),
      Arm.ARM_LINKAGE_2_MASS,
      Arm.ARM_LINKAGE_3_MASS
    );

    Positions.Pose3d pastJ1CG = ArmConfiguration.combineCG(
      pastJ2CG,
      Positions.Pose3d.fromRobotSpace(linkage1CG),
      Arm.ARM_LINKAGE_3_MASS + Arm.ARM_LINKAGE_2_MASS,
      Arm.ARM_LINKAGE_1_MASS
    );

    double joint1CGDistance = joint1Pose3d.minus(pastJ1CG.inRobotSpace()).getTranslation().getNorm();
    double joint1CGAngle = Math.atan((joint1Pose3d.getZ() - pastJ1CG.inRobotSpace().getZ()) / Math.sqrt(Math.pow(joint1Pose3d.getX() - pastJ1CG.inRobotSpace().getX(), 2) + Math.pow(joint1Pose3d.getY() - pastJ1CG.inRobotSpace().getY(), 2)));

    return new FFData(joint1CGDistance, joint1CGAngle, getFirstJointGearRatio());
}

public FFData getJoint2FFData() {
    Pose3d linkage2CG = getLinkage2CG().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);
    Pose3d linkage3CG = getLinkage3CG().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);

    Pose3d joint2Pose3d = getJoint2Pose().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);

    Positions.Pose3d pastJ2CG = ArmConfiguration.combineCG(
      Positions.Pose3d.fromRobotSpace(linkage2CG),
      Positions.Pose3d.fromRobotSpace(linkage3CG),
      Arm.ARM_LINKAGE_2_MASS,
      Arm.ARM_LINKAGE_3_MASS
    );

    double joint2CGDistance = joint2Pose3d.minus(pastJ2CG.inRobotSpace()).getTranslation().getNorm();
    double joint2CGAngle = Math.atan((joint2Pose3d.getZ() - pastJ2CG.inRobotSpace().getZ()) / Math.sqrt(Math.pow(joint2Pose3d.getX() - pastJ2CG.inRobotSpace().getX(), 2) + Math.pow(joint2Pose3d.getY() - pastJ2CG.inRobotSpace().getY(), 2)));

    return new FFData(joint2CGDistance, joint2CGAngle, getSecondJointGearRatio());
}

public FFData getJoint3FFData() {
    Pose3d linkage3CG = getLinkage3CG().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);

    Pose3d joint3Pose3d = getJoint3Pose().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE);

    double joint3CGDistance = joint3Pose3d.minus(linkage3CG).getTranslation().getNorm();
    double joint3CGAngle = Math.atan((joint3Pose3d.getZ() - linkage3CG.getZ()) / Math.sqrt(Math.pow(joint3Pose3d.getX() - linkage3CG.getX(), 2) + Math.pow(joint3Pose3d.getY() - linkage3CG.getY(), 2)));

    return new FFData(joint3CGDistance, joint3CGAngle, getThirdJointGearRatio());
}

/**
 * Returns wheather or not this configuration is valid and within the soft limits
 * If out of the soft limits but moving back towards the soft limits, it is still valid
 * @param currentPos The current position of the arm
 * @return Wheather or not the current configuration is valid and within the soft limits
 */
public boolean validConfig(ArmConfiguration currentPos) {
    if (this.turretPositionRotations > currentPos.turretPositionRotations ? this.turretPositionRotations > Arm.ARM_TURRET_FORWARD_SOFT_LIMIT - VALID_POS_OFFSET : this.turretPositionRotations < Arm.ARM_TURRET_REVERSE_SOFT_LIMIT + VALID_POS_OFFSET) {
        return false;
    }
    if (this.firstJointPositionRotations > currentPos.firstJointPositionRotations ? this.firstJointPositionRotations > Arm.ARM_JOINT_1_FORWARD_SOFT_LIMIT - VALID_POS_OFFSET : this.firstJointPositionRotations < Arm.ARM_JOINT_1_REVERSE_SOFT_LIMIT + VALID_POS_OFFSET) {
        return false;
    }
    if (this.secondJointPositionRotations > currentPos.secondJointPositionRotations ? this.secondJointPositionRotations > Arm.ARM_JOINT_2_FORWARD_SOFT_LIMIT - VALID_POS_OFFSET : this.secondJointPositionRotations < Arm.ARM_JOINT_2_REVERSE_SOFT_LIMIT + VALID_POS_OFFSET) {
        return false;
    }
    if (this.thirdJointPositionRotations > currentPos.thirdJointPositionRotations ? this.thirdJointPositionRotations > Arm.ARM_JOINT_3_FORWARD_SOFT_LIMIT - VALID_POS_OFFSET : this.thirdJointPositionRotations < Arm.ARM_JOINT_3_REVERSE_SOFT_LIMIT + VALID_POS_OFFSET) {
        return false;
    }
    if (this.wristPositionRotations > currentPos.wristPositionRotations ? this.wristPositionRotations > Arm.ARM_WRIST_FORWARD_SOFT_LIMIT - VALID_POS_OFFSET : this.wristPositionRotations < Arm.ARM_WRIST_REVERSE_SOFT_LIMIT + VALID_POS_OFFSET) {
        return false;
    }
    if (getEndPosition().inRobotSpace().getZ() <= 0) {
        return false;
    }
    return true;
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
    + "\n Turret Instantaneous Gear Ratio" + getTurretGearRatio()
    + "\n First Joint Instantaneous Gear Ratio" + getFirstJointGearRatio()
    + "\n Second Joint Instantaneous Gear Ratio" + getSecondJointGearRatio()
    + "\n Third Joint Instantaneous Gear Ratio" + getThirdJointGearRatio()
    + "\n Wrist Instantaneous Gear Ratio" + getWristGearRatio()
    + "\n First Joint FF Data: " + getJoint1FFData()
    + "\n Second Joint FF Data: " + getJoint2FFData()
    + "\n Third Joint FF Data: " + getJoint3FFData()
    + String.format("\nPosition (RS): (%.2f, %.2f, %.2f)", getEndPosition().inRobotSpace().getX(), getEndPosition().inRobotSpace().getY(), getEndPosition().inRobotSpace().getZ())
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
