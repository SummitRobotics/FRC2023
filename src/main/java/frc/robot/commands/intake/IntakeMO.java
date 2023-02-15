package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.drivers.ControllerDriver;
import frc.robot.oi.inputs.OIAxis.PrioritizedAxis;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.lists.AxisPriorities;

public class IntakeMO extends CommandBase {

    Intake intake;
    PrioritizedAxis pivotAxis;
    PrioritizedAxis motorAxis;
    
    public IntakeMO(Intake intake, ControllerDriver controller) {
        this.intake = intake;
        pivotAxis = controller.leftX.prioritize(AxisPriorities.MANUAL_OVERRIDE);
        motorAxis = controller.rightX.prioritize(AxisPriorities.MANUAL_OVERRIDE);
    }

    @Override
    public void execute() {
        intake.setPivotMotor(pivotAxis.get());
        intake.setIntakeMotor(motorAxis.get());
    }

    @Override
    public void end(final boolean interrupted) {
        intake.stop();
        pivotAxis.destroy();
        motorAxis.destroy();
    }
}
