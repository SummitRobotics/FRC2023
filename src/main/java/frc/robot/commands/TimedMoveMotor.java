package frc.robot.commands;

import java.util.function.DoubleConsumer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TimedMoveMotor extends CommandBase {

    DoubleConsumer powerSetter;
    double power;
    double seconds;
    Timer timer;
    
    public TimedMoveMotor(DoubleConsumer powerSetter, double power, double seconds) {
        this.powerSetter = powerSetter;
        this.power = power;
        this.seconds = seconds;
        this.timer = new Timer();
    }

    public TimedMoveMotor(DoubleConsumer powerSetter, double power, double seconds, Subsystem... requirements) {
        this.powerSetter = powerSetter;
        this.power = power;
        this.seconds = seconds;
        this.timer = new Timer();

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        powerSetter.accept(power);
    }

    @Override
    public void end(final boolean interrupted) {
        timer.stop();
        powerSetter.accept(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }
}
