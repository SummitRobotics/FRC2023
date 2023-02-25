// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.utilities.Positions;

public class MoveArmFastUnsafe extends CommandBase {

    private final Arm arm;
    private ArmConfiguration startConfiguration;
    private final ArmConfiguration endConfiguration;

    private static final double FAST_SPEED = 1;
    private static final double SLOW_SPEED = 0.2;
    private static final double CLOSE_DISTANCE = 0.2;
    private static final double TOLERANCE = 5;

    public MoveArmFastUnsafe(Arm arm, Positions.Pose3d endPosition, double grabberAngleRadians, double wristRotationRadians) {
        this.arm = arm;
        this.endConfiguration = ArmConfiguration.fromEndPosition(endPosition, grabberAngleRadians, wristRotationRadians);

        addRequirements(arm);
    }

    public MoveArmFastUnsafe(Arm arm, ArmConfiguration endConfiguration) {
        this.arm = arm;
        this.endConfiguration = endConfiguration;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startConfiguration = arm.getCurrentArmConfiguration();

        double dTurret = endConfiguration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS) - startConfiguration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dFirstJoint = endConfiguration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - startConfiguration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dSecondJoint = endConfiguration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - startConfiguration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dThirdJoint = endConfiguration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - startConfiguration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dWrist = endConfiguration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS) - startConfiguration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS);

        double maxDelta = Math.max(Math.max(Math.max(Math.max(Math.abs(dTurret), Math.abs(dFirstJoint)), Math.abs(dSecondJoint)), Math.abs(dThirdJoint)), Math.abs(dWrist));

        double turretSpeedPercent = dTurret / maxDelta;
        double firstJointSpeedPercent = dFirstJoint / maxDelta;
        double secondJointSpeedPercent = dSecondJoint / maxDelta;
        double thirdJointSpeedPercent = dThirdJoint / maxDelta;
        double wristSpeedPercent = dWrist / maxDelta;

        arm.setTurretMotorVoltage(turretSpeedPercent * FAST_SPEED);
        arm.setJoint1MotorVoltage(firstJointSpeedPercent * FAST_SPEED);
        arm.setJoint2MotorVoltage(secondJointSpeedPercent * FAST_SPEED);
        arm.setJoint3MotorVoltage(thirdJointSpeedPercent * FAST_SPEED);
        arm.setWristMotorVoltage(wristSpeedPercent * FAST_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ArmConfiguration currentConfiguration = arm.getCurrentArmConfiguration();
        double dTurret = endConfiguration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dFirstJoint = endConfiguration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dSecondJoint = endConfiguration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dThirdJoint = endConfiguration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dWrist = endConfiguration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS);

        double maxDelta = Math.max(Math.max(Math.max(Math.max(Math.abs(dTurret), Math.abs(dFirstJoint)), Math.abs(dSecondJoint)), Math.abs(dThirdJoint)), Math.abs(dWrist));
        double minDelta = Math.min(Math.min(Math.min(Math.min(Math.abs(dTurret), Math.abs(dFirstJoint)), Math.abs(dSecondJoint)), Math.abs(dThirdJoint)), Math.abs(dWrist));

        double turretSpeedPercent = dTurret / maxDelta;
        double firstJointSpeedPercent = dFirstJoint / maxDelta;
        double secondJointSpeedPercent = dSecondJoint / maxDelta;
        double thirdJointSpeedPercent = dThirdJoint / maxDelta;
        double wristSpeedPercent = dWrist / maxDelta;

        if (minDelta < CLOSE_DISTANCE) {
            arm.setTurretMotorVoltage(turretSpeedPercent * SLOW_SPEED);
            arm.setJoint1MotorVoltage(firstJointSpeedPercent * SLOW_SPEED);
            arm.setJoint2MotorVoltage(secondJointSpeedPercent * SLOW_SPEED);
            arm.setJoint3MotorVoltage(thirdJointSpeedPercent * SLOW_SPEED);
            arm.setWristMotorVoltage(wristSpeedPercent * SLOW_SPEED);
        } else {
            arm.setTurretMotorVoltage(turretSpeedPercent * FAST_SPEED);
            arm.setJoint1MotorVoltage(firstJointSpeedPercent * FAST_SPEED);
            arm.setJoint2MotorVoltage(secondJointSpeedPercent * FAST_SPEED);
            arm.setJoint3MotorVoltage(thirdJointSpeedPercent * FAST_SPEED);
            arm.setWristMotorVoltage(wristSpeedPercent * FAST_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setTurretMotorVoltage(0);
        arm.setJoint1MotorVoltage(0);
        arm.setJoint2MotorVoltage(0);
        arm.setJoint3MotorVoltage(0);
        arm.setWristMotorVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        ArmConfiguration currentConfiguration = arm.getCurrentArmConfiguration();
        double dTurret = endConfiguration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getTurretPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dFirstJoint = endConfiguration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getFirstJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dSecondJoint = endConfiguration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getSecondJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dThirdJoint = endConfiguration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getThirdJointPosition(POSITION_TYPE.ENCODER_ROTATIONS);
        double dWrist = endConfiguration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS) - currentConfiguration.getWristPosition(POSITION_TYPE.ENCODER_ROTATIONS);

        double maxDelta = Math.max(Math.max(Math.max(Math.max(Math.abs(dTurret), Math.abs(dFirstJoint)), Math.abs(dSecondJoint)), Math.abs(dThirdJoint)), Math.abs(dWrist));
        return maxDelta < TOLERANCE;
    }
}
