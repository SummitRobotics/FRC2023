package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.inputs.OIAxis.PrioritizedAxis;
import frc.robot.oi.inputs.OITrigger.PrioritizedTrigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmConfiguration;
import frc.robot.subsystems.Arm.ArmConfiguration.POSITION_TYPE;
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

    public ArmMO(Arm arm, ControllerDriver controller) {
        this.arm = arm;
        endPose = arm.getCurrentArmConfiguration().getEndPosition();
        grabberRadians
            = arm.getCurrentArmConfiguration().getThirdJointPosition(POSITION_TYPE.ANGLE);
        wristRadians 
            = arm.getCurrentArmConfiguration().getWristPosition(POSITION_TYPE.ANGLE);
        xAxis = controller.leftX.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        yAxis = controller.leftY.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        zAxis = controller.rightY.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        grabberUp = controller.rightBumper.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        grabberDown = controller.leftBumper.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        wristLeft = controller.leftTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        wristRight = controller.rightTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        clampButton = controller.buttonA.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        addRequirements(arm);
    }

    @Override
    public void execute() {

        // TODO - check if input dampening is correct
        // convert from robot space, edit based on inputs, and convert back to robot space
        endPose = Positions.Pose3d.fromRobotSpace(
            new Pose3d(
                endPose.inRobotSpace().getX() + xAxis.get() / 50,
                endPose.inRobotSpace().getY() + yAxis.get() / 50,
                endPose.inRobotSpace().getZ() + zAxis.get() / 50,
                endPose.inRobotSpace().getRotation() // It doesn't matter what this rotation is.
            )
        );

        grabberRadians
            = grabberRadians + (grabberUp.get() ? 0.02 : 0) - (grabberDown.get() ? 0.02 : 0);
        wristRadians = wristRadians + wristLeft.get() / 50 - wristRight.get() / 50;

        arm.setToConfiguration(
            ArmConfiguration.fromEndPosition(endPose, grabberRadians, wristRadians));
    }
}
