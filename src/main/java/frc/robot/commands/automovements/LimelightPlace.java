package frc.robot.commands.automovements;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class LimelightPlace extends CommandBase {

    private PIDController pidController = new PIDController(0.02
    , 0, 0);
    private Drivetrain drivetrain;
    private Arm arm;

    // TODO - change to use our "Lemonlight" wrapper class instead of directly reading networktables
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry tv;
    
    public LimelightPlace(Drivetrain drivetrain, Arm arm) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        tv = table.getEntry("tv");
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pidController.reset();
        pidController.setTolerance(0.25, 1);
        pidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (tv.getDouble(0) == 1) { // if we have a target
            double turretPos = arm.getCurrentArmConfiguration().getTurretPosition(POSITION_TYPE.ANGLE);
            drivetrain.setBothMotorPower(
                turretPos < 0 ? 1 : -1
                * pidController.calculate(tx.getDouble(0.0) + (turretPos < 0 ? -2.25 : 2.25))
            );
        }
    }

    @Override
    public void end(final boolean interrupted) {
        drivetrain.stop();
    }
}
