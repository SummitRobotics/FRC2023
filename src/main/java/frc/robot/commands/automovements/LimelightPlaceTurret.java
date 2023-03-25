package frc.robot.commands.automovements;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.inputs.OITrigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class LimelightPlaceTurret extends CommandBase {

    // private PIDController pidController = new PIDController(0.2
    // , 0, 0);
    private Arm arm;

    // TODO - change to use our "Lemonlight" wrapper class instead of directly reading networktables
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry tv;

    private final OITrigger leftButton;
    private final OITrigger rightButton;
    private double manualOffset = 0;
    
    public LimelightPlaceTurret(Arm arm, OITrigger leftButton, OITrigger rightButton) {
        this.arm = arm;
        this.leftButton = leftButton;
        this.rightButton = rightButton;
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        tv = table.getEntry("tv");
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // pidController.reset();
        // pidController.setTolerance(0.25, 1);
        // pidController.setSetpoint(0);
        manualOffset = 0;
    }

    @Override
    public void execute() {
        if (leftButton.getTrigger().getAsBoolean()) {
          manualOffset += 0.05;
        }
        if (rightButton.getTrigger().getAsBoolean()) {
          manualOffset += -0.05;
        }

        if (tv.getDouble(0) == 1) { // if we have a target
            ArmConfiguration currentArmConfig = arm.getCurrentArmConfiguration();
            double turretAngle = currentArmConfig.getTurretPosition(POSITION_TYPE.ANGLE);
            double dAngle = -Math.toRadians((tx.getDouble(0.0) * 0.3) + (turretAngle < 0 ? -2.25 : 2.25) + manualOffset);
            double pidafiledAngle = dAngle;
            ArmConfiguration newArmConfig = new ArmConfiguration(
            turretAngle + pidafiledAngle,
            currentArmConfig.getFirstJointPosition(POSITION_TYPE.ANGLE),
            currentArmConfig.getSecondJointPosition(POSITION_TYPE.ANGLE),
            currentArmConfig.getThirdJointPosition(POSITION_TYPE.ANGLE), 
            POSITION_TYPE.ANGLE);
            arm.setToConfiguration(newArmConfig);
      }
    }

    @Override
    public void end(final boolean interrupted) {
        arm.stop();
    }
}
