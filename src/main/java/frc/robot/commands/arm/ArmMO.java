package frc.robot.commands.arm;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.inputs.OIAxis.PrioritizedAxis;
import frc.robot.oi.inputs.OITrigger.PrioritizedTrigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import frc.robot.utilities.Positions;
import frc.robot.utilities.lists.AxisPriorities;

public class ArmMO extends CommandBase {

    Arm arm;
    Positions.Pose3d endPose;
    double grabberRadians;
    double wristRadians;
    PrioritizedAxis xAxis;
    PrioritizedAxis yAxis;
    PrioritizedAxis zAxis;
    PrioritizedTrigger clampButton;
    PrioritizedTrigger grabberUp;
    PrioritizedTrigger grabberDown;
    PrioritizedAxis wristLeft;
    PrioritizedAxis wristRight;

    ControllerDriver controller;

    public ArmMO(Arm arm, ControllerDriver controller) {
        this.arm = arm;
        this.controller = controller;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        xAxis = controller.leftX.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        yAxis = controller.leftY.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        zAxis = controller.rightY.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        grabberUp = controller.rightBumper.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        grabberDown = controller.leftBumper.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        wristLeft = controller.leftTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        wristRight = controller.rightTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        clampButton = controller.buttonA.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        endPose = arm.getTargetArmConfiguration().getEndPosition();
        grabberRadians
            = arm.getTargetArmConfiguration().getThirdJointPosition(POSITION_TYPE.ANGLE);
        wristRadians 
            = arm.getTargetArmConfiguration().getWristPosition(POSITION_TYPE.ANGLE);
    }

    @Override
    public void execute() {

        // convert from robot space, edit based on inputs, and convert back to robot space
        endPose = Positions.Pose3d.fromRobotSpace(
            new Pose3d(
                endPose.inRobotSpace().getX() + -yAxis.get() / 200,
                // endPose.inRobotSpace().getY() + yAxis.get() / 200,
                endPose.inRobotSpace().getY(),
                endPose.inRobotSpace().getZ() + -zAxis.get() / 200,
                endPose.inRobotSpace().getRotation() // It doesn't matter what this rotation is.
            )
        );

        grabberRadians = grabberRadians + (grabberUp.get() ? 0.01 : 0) - (grabberDown.get() ? 0.01 : 0);
        wristRadians = wristRadians + wristLeft.get() / 100 - wristRight.get() / 100;

        arm.setToConfiguration(
            ArmConfiguration.fromEndPosition(endPose, grabberRadians, wristRadians));

        if (clampButton.getTrigger().debounce(0.1, DebounceType.kRising).getAsBoolean()) {
            if (arm.getClampSolenoidState()) arm.unclamp(); else arm.clamp(); 
        }
    }

    @Override
    public void end(final boolean interrupted) {
        xAxis.destroy();
        yAxis.destroy();
        zAxis.destroy();
        clampButton.destroy();
        grabberUp.destroy();
        grabberDown.destroy();
        wristLeft.destroy();
        wristRight.destroy();
        arm.stop();
    }
}
