package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.inputs.OIAxis.PrioritizedAxis;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utilities.lists.AxisPriorities;

public class FullManualArm extends CommandBase {

    public enum Type {
        TURRET,
        JOINT_1,
        JOINT_2,
        JOINT_3,
        WRIST
    }

    Arm arm;
    Type type;
    PrioritizedAxis leftTriggerAxis;
    PrioritizedAxis rightTriggerAxis;
    ControllerDriver controller;

    public FullManualArm(Arm arm, Type type, ControllerDriver controller) {
        this.arm = arm;
        this.type = type;
        this.controller = controller;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.leftTriggerAxis = controller.leftTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        this.rightTriggerAxis = controller.rightTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        if (type == Type.TURRET) {
            // arm.setTurretSoftLimit(false);
        } else if (type == Type.JOINT_1) {
            // arm.setFirstJointSoftLimit(false);
        } else if (type == Type.JOINT_2) {
            // arm.setSecondJointSoftLimit(false);
        } else if (type == Type.JOINT_3) {
            // arm.setThirdJointSoftLimit(false);
        } else if (type == Type.WRIST) {
            // arm.setWristSoftLimit(false);
        }
    }

    @Override
    public void execute() {
        if (type == Type.TURRET) {
            arm.setTurretMotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        } else if (type == Type.JOINT_1) {
            arm.setJoint1MotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        } else if (type == Type.JOINT_2) {
            arm.setJoint2MotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        } else if (type == Type.JOINT_3) {
            arm.setJoint3MotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        } else if (type == Type.WRIST) {
            arm.setWristMotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        }
    }

    @Override
    public void end(final boolean interrupted) {
        leftTriggerAxis.destroy();
        rightTriggerAxis.destroy();
        if (type == Type.TURRET) {
            arm.setTurretMotorVoltage(0);
            // arm.setTurretSoftLimit(true);
        } else if (type == Type.JOINT_1) {
            arm.setJoint1MotorVoltage(0);
            // arm.setFirstJointSoftLimit(true);
        } else if (type == Type.JOINT_2) {
            arm.setJoint2MotorVoltage(0);
            // arm.setSecondJointSoftLimit(true);
        } else if (type == Type.JOINT_3) {
            arm.setJoint3MotorVoltage(0);
            // arm.setThirdJointSoftLimit(true);
        } else if (type == Type.WRIST) {
            arm.setWristMotorVoltage(0);
            // arm.setWristSoftLimit(true);
        }
    }
}
