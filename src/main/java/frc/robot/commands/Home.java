package frc.robot.commands;

import java.util.ArrayList;
import java.util.PriorityQueue;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.homing.HomeableCANSparkMax;
import frc.robot.utilities.homing.HomeableSubsystem;
import frc.robot.utilities.homing.HomeableCANSparkMax.Type;

public class Home extends CommandBase {

    // maps priorities to maps that contain homeables and whether or not they are finished
    private PriorityQueue<HomeableCANSparkMax> homeables = new PriorityQueue<>(HomeableCANSparkMax.ORDER_COMPARATOR);
    private ArrayList<HomeableCANSparkMax> activeHoming = new ArrayList<HomeableCANSparkMax>();

    public Home(HomeableCANSparkMax... toHome) {

        for (HomeableCANSparkMax homeable : toHome) {
            homeables.add(homeable);
            addRequirements(homeable.getSubsystemObject());
        }
    }

    public Home(HomeableSubsystem subsystem) {
        this(subsystem.getHomeables());
    }

    @Override
    public void initialize() {
        for (HomeableCANSparkMax homeable : homeables) {
            // Is it a good idea to use triggers for this?
            new Trigger(() -> homeable.isFinished()).onTrue(new InstantCommand(() -> homeable.end()));
        }
    }

    @Override
    public void execute() {

        boolean areFinished = true;
        for (HomeableCANSparkMax homeable : activeHoming) {
            homeable.updateStopCondition();
            areFinished &= homeable.isFinished();
        }

        if (areFinished) {
            activeHoming.clear();
            if (!homeables.isEmpty()) {

                while (
                    activeHoming.isEmpty()
                    || homeables.peek().getOrder() == activeHoming.get(0).getOrder()
                ) {
                    HomeableCANSparkMax homeable = homeables.poll();
                    activeHoming.add(homeable);
                    homeable.init();
                }
            }
        }

        for (HomeableCANSparkMax homeable : activeHoming) {
            if (homeable.getType() == Type.ByCurrent) homeable.updateCurrent();
            if (homeable.isFinished()) homeable.setPower(0); else homeable.setPower();
        }
    }

    public boolean isFinished() {
        return activeHoming.isEmpty() && homeables.isEmpty();
    }
}
