package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDRange;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.oi.inputs.OIAxis;
import frc.robot.oi.inputs.OITrigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.ChangeRateLimiter;
import frc.robot.utilities.Functions;
import frc.robot.utilities.RollingAverage;
import frc.robot.utilities.lists.AxisPriorities;
import frc.robot.utilities.lists.Colors;
import frc.robot.utilities.lists.LEDPriorities;

/**
 * Command for Arcade Drive.
 */
public class ArcadeDrive extends CommandBase {

    private final Drivetrain drivetrain;

    private double forwardPower;
    private double reversePower;
    private boolean activateSwitchfoot;

    private final OIAxis.PrioritizedAxis forwardPowerAxis;
    private final OIAxis.PrioritizedAxis reversePowerAxis;
    private final OIAxis.PrioritizedAxis turnAxis;

    private final ChangeRateLimiter limiter;

    private static final double DEAD_ZONE = .01;

    private static final double MAX_CHANGE_RATE = 0.05;

    private final RollingAverage avgSpeed = new RollingAverage(2, true);

    private final RollingAverage avgPower = new RollingAverage(2, true);

    private final boolean isSingleAxis;

    private boolean ledsOn = false;

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
        this.forwardPowerAxis = forwardPowerAxis.prioritize(AxisPriorities.DRIVE);
        this.reversePowerAxis = reversePowerAxis.prioritize(AxisPriorities.DRIVE);
        this.turnAxis = turnAxis.prioritize(AxisPriorities.DRIVE);

        limiter = new ChangeRateLimiter(MAX_CHANGE_RATE);

        addRequirements(drivetrain);
        isSingleAxis = false;
        ledsOn = false;

        switchfoot.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(
            new InstantCommand(() -> {
                activateSwitchfoot = !activateSwitchfoot;
                if (activateSwitchfoot) {
                    LEDs.getInstance().addCall("reversed", new LEDCall(LEDPriorities.DRIVE_REV, LEDRange.Aarms).ffh(Colors.YELLOW, Colors.OFF));  
                } else {
                    LEDs.getInstance().removeCall("reversed");
                }
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
        this.forwardPowerAxis = powerAxis.prioritize(AxisPriorities.DRIVE);
        this.reversePowerAxis = null;
        this.turnAxis = turnAxis.prioritize(AxisPriorities.DRIVE);

        limiter = new ChangeRateLimiter(MAX_CHANGE_RATE);

        addRequirements(drivetrain);
        isSingleAxis = true;
        ledsOn = false;

        switchfoot.prioritize(AxisPriorities.DRIVE).getTrigger().onTrue(
            new InstantCommand(() -> {
                activateSwitchfoot = !activateSwitchfoot;
                if (activateSwitchfoot) {
                    LEDs.getInstance().addCall("reversed", new LEDCall(LEDPriorities.DRIVE_REV, LEDRange.Aarms).ffh(Colors.YELLOW, Colors.OFF));  
                } else {
                    LEDs.getInstance().removeCall("reversed");
                }
            }
        ));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setOpenRampRate(0);
        avgPower.reset();
        avgSpeed.reset();
        activateSwitchfoot = false;
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

            power = forwardPower - reversePower;
        }

        power = limiter.getRateLimitedValue(power);

        avgPower.update(Math.abs(power));

        avgSpeed.update(Math.abs(drivetrain.getRightRPM()) + Math.abs(drivetrain.getLeftRPM()));

        //shifts into low gear if drivetrain stalled
        if ((avgPower.getAverage() > .5) && avgSpeed.getAverage() < 15) {
            drivetrain.lowGear();
        }

        double turn = Math.pow(turnAxis.get(), 3);

        //System.out.println(turn);

        if (activateSwitchfoot) {
            //dumb stuff is dumb
            if (!ledsOn) {
                LEDs.getInstance().addCall("reversed", new LEDCall(LEDPriorities.DRIVE_REV, LEDRange.Aarms).ffh(Colors.YELLOW, Colors.OFF));   
            }
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
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}