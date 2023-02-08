package frc.robot.commands.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Functions;

public class ChargeStationBalance extends CommandBase {

    private final double
        P = 0.008,
        I = 0, // do not change this
        D = 0.0015;
    
    private final Drivetrain drivetrain;
    private final AHRS gyro;
    private PIDController controller;

    private boolean isAtPlatform;

    public ChargeStationBalance(Drivetrain drivetrain, AHRS gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.controller = new PIDController(P, I, D);

        isAtPlatform = false;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setBothMotorPower(0.5);
        controller.reset();
        controller.setTolerance(0.1, 1);
        controller.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (gyro.getRoll() > 5 && !isAtPlatform) {
            isAtPlatform = true;
        }

        if (isAtPlatform) {
            double power = -Functions.clampDouble(
                controller.calculate(gyro.getRoll() + gyro.getPitch()),
                0.75,
                -0.75);
            drivetrain.setBothMotorPower(power);
        }
    }

    @Override
    public void end(final boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
