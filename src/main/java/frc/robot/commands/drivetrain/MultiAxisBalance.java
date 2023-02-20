package frc.robot.commands.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;


/**
 * Centers on the gyroscope axis that is moving and balances from there.
 */
public class MultiAxisBalance extends CommandBase {

    private final double
        P = 0.008,
        I = 0, // do not change this
        D = 0.0015;

    Drivetrain drivetrain;
    AHRS gyro;
    private PIDController controller;
    private boolean isAtPlatform;

    public MultiAxisBalance(Drivetrain drivetrain, AHRS gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.controller = new PIDController(P, I, D);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.reset();
        controller.setTolerance(0.1, 1);
        controller.setSetpoint(0);
        isAtPlatform = false;
        drivetrain.setBothMotorPower(0.5);
    }

    @Override
    public void execute() {

        if (!isAtPlatform && gyro.getRoll() + gyro.getPitch() > 5) isAtPlatform = true;
        if (isAtPlatform) {
            if (gyro.getPitch() < -3) {
                drivetrain.setRightMotorPower(0.3);
                drivetrain.setLeftMotorPower(-0.3);
            } else if (gyro.getPitch() > 3) {
                drivetrain.setRightMotorPower(-0.3);
                drivetrain.setLeftMotorPower(0.3);
            } else {
                double power = -Functions.clampDouble(
                    controller.calculate(gyro.getRoll() + gyro.getPitch()), 0.75, -0.75);
                drivetrain.setBothMotorPower(power);
            }
        }
    }

    @Override
    public void end(final boolean interrupted) {
        drivetrain.stop();
    }
}
