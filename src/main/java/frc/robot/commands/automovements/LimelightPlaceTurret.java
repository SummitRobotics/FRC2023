package frc.robot.commands.automovements;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private double dAngle = Double.MAX_VALUE;
    private static final double THRESHOLD = Math.toRadians(1);
    private static final int THRESHOLD_COUNT = 5;
    private int thresholdCount = 0;
    
    public LimelightPlaceTurret(Arm arm) {
        this.arm = arm;
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
        dAngle = Double.MAX_VALUE;
        thresholdCount = 0;
    }

    @Override
    public void execute() {

        if (tv.getDouble(0) == 1) { // if we have a target
            ArmConfiguration currentArmConfig = arm.getCurrentArmConfiguration();
            double turretAngle = currentArmConfig.getTurretPosition(POSITION_TYPE.ANGLE);
            dAngle = -Math.toRadians((tx.getDouble(0.0) * 0.2) + (turretAngle < 0 ? -2.25 : 2.25));
            ArmConfiguration newArmConfig = new ArmConfiguration(
            turretAngle + dAngle,
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

    @Override
    public boolean isFinished() {
        if (Math.abs(dAngle) < THRESHOLD) {
            thresholdCount++;
        } else {
            thresholdCount = 0;
        }

        return thresholdCount > THRESHOLD_COUNT;
    }
}
