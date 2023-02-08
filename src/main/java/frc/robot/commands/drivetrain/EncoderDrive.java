package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to move the drivetrain with the encoders.
 */
public class EncoderDrive extends CommandBase {

    private final Drivetrain drivetrain;
    private double left;
    private double right;
    private double maximumPower = 0.5;
    /**
     * drives each whele the set dist using encoders, regardless of gear
     * 
     * max output is 0.5 unless specifyed
     *
     * @param drivetrain the robot's drivetrain
     * @param left The distance to move the left side
     * @param right The distance to move the right side
     */

    public EncoderDrive(double left, double right, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;

        addRequirements(drivetrain);
    }
    /**
     * drives each whele the set dist using encoders, regardless of gear
     *
     * @param drivetrain the robot's drivetrain
     * @param left The distance to move the left side
     * @param right The distance to move the right side
     * @param maximumPower The maximum power the PID can set
     */

    public EncoderDrive(double left, double right, Drivetrain drivetrain, double maximumPower) {
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;
        this.maximumPower = maximumPower;
        addRequirements(drivetrain);
    }
    // Called when the command is initially scheduled.

    @Override
    public void initialize() {
        drivetrain.stop();
        drivetrain.setPIDMaxPower(maximumPower);
        drivetrain.zeroDistance();
        
        // TODO test if this is accurate
        drivetrain.setLeftMotorTarget(drivetrain.distToEncoder(left));
        
        drivetrain.setRightMotorTarget(drivetrain.distToEncoder(right));
    }

    @Override
    public void execute() {
        System.out.println(drivetrain.getLeftDistance() - left);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getLeftEncoderPosition() - drivetrain.distToEncoder(left)) < 2
            && Math.abs(drivetrain.getRightEncoderPosition() - drivetrain.distToEncoder(right)) < 2;

    }
}
