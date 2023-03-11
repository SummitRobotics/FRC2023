package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OITrigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.AxisPriorities;

/**
 * Command for Arcade Drive.
 */
public class ArcadeDrive extends CommandBase {

    private final Drivetrain drivetrain;

    private double forwardPower;
    private double reversePower;
    private boolean activateSwitchfoot;

    private final OIAxis forwardPowerAxis;
    private final OIAxis reversePowerAxis;
    private final OIAxis turnAxis;

    private OIAxis.PrioritizedAxis forwardPowerAxisPrio;
    private OIAxis.PrioritizedAxis reversePowerAxisPrio;
    private OIAxis.PrioritizedAxis turnAxisPrio;

    private final ChangeRateLimiter powerLimiter;

    private static final double DEAD_ZONE = .01;

    private static final double MAX_CHANGE_RATE = 0.05;

    private final RollingAverage avgSpeed = new RollingAverage(2, true);

    private final RollingAverage avgPower = new RollingAverage(2, true);

    private final boolean isSingleAxis;

    /**
     * teleop driver control.
     *
     * @param drivetrain       drivetrain instance
     * @param forwardPowerAxis control axis for forward power
     * @param reversePowerAxis control axis for reverse power
     * @param turnAxis         control axis for the drivetrain turn
     * @param switchfoot         drive in reverse
     */
    public ArcadeDrive(
        Drivetrain drivetrain,
        OIAxis forwardPowerAxis, 
        OIAxis reversePowerAxis, 
        OIAxis turnAxis,
        OITrigger switchfoot) {
        this.drivetrain = drivetrain;
        this.forwardPowerAxis = forwardPowerAxis;
        this.reversePowerAxis = reversePowerAxis;
        this.turnAxis = turnAxis;

        powerLimiter = new ChangeRateLimiter(MAX_CHANGE_RATE);

        addRequirements(drivetrain);
        isSingleAxis = false;

        switchfoot.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(
            new InstantCommand(() -> {
                activateSwitchfoot = !activateSwitchfoot;
            }
        ));
    }

    /**
     * teleop driver control.
     *
     * @param drivetrain drivetrain instance
     * @param powerAxis  control axis for the drivetrain power
     * @param turnAxis   control axis for the drivetrain turn
     * @param switchfoot swaps direction
     */
    public ArcadeDrive(
        Drivetrain drivetrain, 
        OIAxis powerAxis, 
        OIAxis turnAxis, 
        OITrigger switchfoot) {
        this.drivetrain = drivetrain;
        this.forwardPowerAxis = powerAxis;
        this.reversePowerAxis = null;
        this.turnAxis = turnAxis;

        powerLimiter = new ChangeRateLimiter(MAX_CHANGE_RATE);

        addRequirements(drivetrain);
        isSingleAxis = true;

        switchfoot.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(
            new InstantCommand(() -> {
                activateSwitchfoot = !activateSwitchfoot;
            }
        ));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        forwardPowerAxisPrio = forwardPowerAxis.prioritize(AxisPriorities.DRIVE);
        if (reversePowerAxis != null) {
            reversePowerAxisPrio = reversePowerAxis.prioritize(AxisPriorities.DRIVE);
        } else {
            reversePowerAxisPrio = null;
        }
        turnAxisPrio = turnAxis.prioritize(AxisPriorities.DRIVE);
        drivetrain.setOpenRampRate(0);
        avgPower.reset();
        avgSpeed.reset();
        activateSwitchfoot = false;
        powerLimiter.resetOld();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double power;

        if (isSingleAxis) {
            power = Math.pow(Functions.deadzone(DEAD_ZONE, forwardPowerAxis.get()), 3);
        } else {
            forwardPower = forwardPowerAxis.get();
            reversePower = reversePowerAxis.get();   

            forwardPower = Functions.deadzone(DEAD_ZONE, forwardPower);
            reversePower = Functions.deadzone(DEAD_ZONE, reversePower);

            forwardPower = Math.pow(forwardPower, 2);
            reversePower = Math.pow(reversePower, 2);

            power = powerLimiter.getRateLimitedValue(forwardPower - reversePower);
        }

        avgPower.update(Math.abs(power));

        avgSpeed.update(Math.abs(drivetrain.getRightRPM()) + Math.abs(drivetrain.getLeftRPM()));

        //shifts into low gear if drivetrain stalled
        if ((avgPower.getAverage() > .5) && avgSpeed.getAverage() < 15) {
            drivetrain.lowGear();
        }

        System.out.println(turnAxis.get());
        
        double turnVal = 0;
        
        if (Math.abs(turnAxis.get()) > 0.845) {
            turnVal = Math.pow(turnAxis.get(), 3) * 0.7;
        } else {
            turnVal = 0.5 * turnAxis.get();
        }
        
        double turn = -drivetrain.turnSpeedToMotorPower(turnVal);
        // turn = -drivetrain.turnSpeedToMotorPower(Math.pow(turnAxis.get(), 3));
        // turn = -drivetrain.turnSpeedToMotorPower(0.7 * Math.pow(turnAxis.get(), 3));
        // turn = -Math.pow(turnAxis.get(), 3);
        
        if (activateSwitchfoot) {
            turn = -turn;
        }

        // calculates power to the motors
        double leftPower = power + turn;
        double rightPower = power - turn;

        if (!activateSwitchfoot) {
            drivetrain.setLeftMotorPower(leftPower);
            drivetrain.setRightMotorPower(rightPower);
        } else {
            drivetrain.setLeftMotorPower(-leftPower);
            drivetrain.setRightMotorPower(-rightPower);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        LEDs.getInstance().removeCall("reversed");
        forwardPowerAxisPrio.destroy();
        if (reversePowerAxisPrio != null) {
            reversePowerAxisPrio.destroy();
        }
        turnAxisPrio.destroy();
    }
}