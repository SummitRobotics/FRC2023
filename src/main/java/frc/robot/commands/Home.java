package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.homing.HomeableCANSparkMax;
import frc.robot.utilities.homing.HomeableSubsystem;
import frc.robot.utilities.homing.HomeableCANSparkMax.Type;

public class Home extends CommandBase {
    
    private HomeableCANSparkMax[] homeables;

    public Home(HomeableSubsystem subsystem) {

        homeables = subsystem.getHomeables();
        addRequirements(subsystem.getSubsystemObject());
    }

    public Home(HomeableCANSparkMax... homeables) {

        this.homeables = homeables;
        for (HomeableCANSparkMax homeable : homeables) {
            addRequirements(homeable.getSubsystemObject());
        }
    }

    @Override
    public void initialize() {
        for (HomeableCANSparkMax homeable : homeables) {
            if (homeable.getType() == Type.ByCurrent) homeable.resetCurrent();
            if (homeable.hasSoftLimits()) homeable.disableSoftLimits();
        }
    }

    @Override
    public void execute() {
        for (HomeableCANSparkMax homeable : homeables) {

            if (homeable.getType() == Type.ByCurrent) homeable.updateCurrent();
            if (homeable.stopCondition()) homeable.setIsFinished();
            if (homeable.getIsFinished()) homeable.setPower(0); else homeable.setPower();
        }
    }

    @Override
    public void end(final boolean interrupted) {
        for (HomeableCANSparkMax homeable: homeables) {
            homeable.setPower(0);
            if (homeable.hasSoftLimits()) homeable.enableSoftLimits();
        }

        if (!interrupted) {
            for (HomeableCANSparkMax homeable: homeables) {
                homeable.zeroHome();
            } 
        }
    }

    @Override
    public boolean isFinished() {
        boolean result = true;

        for (HomeableCANSparkMax homeable : homeables) {
            result &= homeable.getIsFinished();
        }

        return result;
    }
}
