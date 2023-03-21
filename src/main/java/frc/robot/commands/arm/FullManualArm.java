package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.LEDs.LEDCalls;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.inputs.OIAxis.PrioritizedAxis;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.utilities.lists.AxisPriorities;

public class FullManualArm extends CommandBase {

    public enum Type {
        TURRET,
        JOINT_1,
        JOINT_2,
        JOINT_3,
        INTAKE
    }

    Arm arm;
    ArmIntake intake;
    Type type;
    PrioritizedAxis leftTriggerAxis;
    PrioritizedAxis rightTriggerAxis;
    ControllerDriver controller;

    public FullManualArm(Arm arm, ArmIntake intake, Type type, ControllerDriver controller) {
        this.arm = arm;
        this.intake = intake;
        this.type = type;
        this.controller = controller;

        if (type == Type.INTAKE) {
            addRequirements(intake);
        }
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
        }
        LEDCalls.MO.activate();
    }

    @Override
    public void execute() {
        if (type == Type.TURRET) {
            arm.setTurretMotorVoltage((12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get()) * -1);
        } else if (type == Type.JOINT_1) {
            arm.setJoint1MotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        } else if (type == Type.JOINT_2) {
            arm.setJoint2MotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        } else if (type == Type.JOINT_3) {
            arm.setJoint3MotorVoltage(12 * leftTriggerAxis.get() - 12 * rightTriggerAxis.get());
        } else if (type == Type.INTAKE) {
            intake.setSpeed(leftTriggerAxis.get() - rightTriggerAxis.get());
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
        } else if (type == Type.INTAKE) {
            intake.stop();
        }
        LEDCalls.MO.cancel();
    }
}
