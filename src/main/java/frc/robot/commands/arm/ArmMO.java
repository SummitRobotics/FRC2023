package frc.robot.commands.arm;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.devices.LEDs.LEDCalls;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.drivers.LaunchpadDriver;
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
    PrioritizedAxis wristUp;
    PrioritizedAxis wristDown;

    ControllerDriver controller;
    LaunchpadDriver launchpad;

    public ArmMO(Arm arm, ControllerDriver controller, LaunchpadDriver launchpad) {
        this.arm = arm;
        this.controller = controller;
        this.launchpad = launchpad;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        xAxis = controller.leftX.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        yAxis = controller.leftY.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        zAxis = controller.rightY.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        grabberUp = controller.rightBumper.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        grabberDown = controller.leftBumper.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        wristUp = controller.leftTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        wristDown = controller.leftTrigger.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        clampButton = controller.buttonA.prioritize(AxisPriorities.MANUAL_OVERRIDE);

        endPose = arm.getCurrentArmConfiguration().getEndPosition();
        grabberRadians
            = -arm.getCurrentArmConfiguration().getEndPosition().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE).getRotation().getY();
        wristRadians 
            = arm.getCurrentArmConfiguration().getWristPosition(POSITION_TYPE.ANGLE);

        arm.setToConfiguration(arm.getCurrentArmConfiguration());

        LEDCalls.MO.activate();
    }

    @Override
    public void execute() {

        // convert from robot space, edit based on inputs, and convert back to robot space
        if (launchpad.funRight.getTrigger().getAsBoolean()) {
            System.out.println("fun right");
            endPose = Positions.Pose3d.fromFieldSpace(
                new Pose3d(
                    endPose.inFieldSpace().getX() + -yAxis.get() / 200,
                    endPose.inFieldSpace().getY() + -xAxis.get() / 200,
                    endPose.inFieldSpace().getZ() + -zAxis.get() / 200,
                    endPose.inFieldSpace().getRotation() // It doesn't matter what this rotation is.
                )
            );
        }

        if (launchpad.funMiddle.getTrigger().getAsBoolean()) {
            System.out.println("fun middle");
            endPose = Positions.Pose3d.fromRobotSpace(
                new Pose3d(
                    endPose.inRobotSpace().getX() + -yAxis.get() / 200,
                    endPose.inRobotSpace().getY() + -xAxis.get() / 200,
                    endPose.inRobotSpace().getZ() + -zAxis.get() / 200,
                    endPose.inRobotSpace().getRotation() // It doesn't matter what this rotation is.
                )
            );
        }

        if (launchpad.funLeft.getTrigger().getAsBoolean()) {
            System.out.println("fun left");
            Transform3d thing = new Transform3d(arm.getCurrentArmConfiguration().getEndPosition().inRobotSpace().getTranslation(),arm.getCurrentArmConfiguration().getEndPosition().inRobotSpace().getRotation());
            endPose = Positions.Pose3d.fromOtherSpace(
                new Pose3d(
                    endPose.inOtherSpace(thing).getX() + -yAxis.get() / 200,
                    endPose.inOtherSpace(thing).getY() + -xAxis.get() / 200,
                    endPose.inOtherSpace(thing).getZ() + -zAxis.get() / 200,
                    endPose.inOtherSpace(thing).getRotation() // It doesn't matter what this rotation is.
                ),
                thing
            );
        }

        grabberRadians = grabberRadians + (grabberUp.get() ? 0.01 : 0) - (grabberDown.get() ? 0.01 : 0);
        wristRadians = wristRadians + wristUp.get() / 100 - wristDown.get() / 100;

        arm.setToConfiguration(
            ArmConfiguration.fromEndPosition(endPose, grabberRadians, wristRadians));

        if (clampButton.getTrigger().debounce(0.1, DebounceType.kRising).getAsBoolean()) {
            if (arm.getClampSolenoidState()) arm.unclamp(); else arm.clamp(); 
        }

        endPose = arm.getTargetArmConfiguration().getEndPosition();
        grabberRadians
            = -arm.getTargetArmConfiguration().getEndPosition().inOtherSpace(Arm.ROBOT_TO_TURRET_BASE).getRotation().getY();
        wristRadians 
            = arm.getTargetArmConfiguration().getWristPosition(POSITION_TYPE.ANGLE);
    }

    @Override
    public void end(final boolean interrupted) {
        xAxis.destroy();
        yAxis.destroy();
        zAxis.destroy();
        clampButton.destroy();
        grabberUp.destroy();
        grabberDown.destroy();
        wristUp.destroy();
        wristDown.destroy();
        arm.stop();
        LEDCalls.MO.cancel();
    }
}
