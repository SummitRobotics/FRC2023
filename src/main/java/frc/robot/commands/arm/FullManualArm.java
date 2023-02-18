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
    PrioritizedAxis plusAxis;
    PrioritizedAxis minusAxis;

    public FullManualArm(Arm arm, Type type, ControllerDriver controller) {
        this.arm = arm;
        this.type = type;
        this.plusAxis = controller.leftTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        this.minusAxis = controller.rightTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (type == Type.TURRET) {
            arm.setTurretMotorVoltage(12 * plusAxis.get() - 12 * minusAxis.get());
        } else if (type == Type.JOINT_1) {
            arm.setJoint1MotorVoltage(12 * plusAxis.get() - 12 * minusAxis.get());
        } else if (type == Type.JOINT_2) {
            arm.setJoint2MotorVoltage(12 * plusAxis.get() - 12 * minusAxis.get());
        } else if (type == Type.JOINT_3) {
            arm.setJoint3MotorVoltage(12 * plusAxis.get() - 12 * minusAxis.get());
        } else if (type == Type.WRIST) {
            arm.setWristMotorVoltage(12 * plusAxis.get() - 12 * minusAxis.get());
        }
    }

    @Override
    public void end(final boolean interrupted) {
        plusAxis.destroy();
        minusAxis.destroy();
    }
}
