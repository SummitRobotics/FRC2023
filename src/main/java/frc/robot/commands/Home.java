package frc.robot.commands;

import java.util.ArrayList;
import java.util.PriorityQueue;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.homing.HomeableCANSparkMax;
import frc.robot.utilities.homing.HomeableSubsystem;
import frc.robot.utilities.homing.HomeableCANSparkMax.Type;

public class Home extends CommandBase {

    // maps priorities to maps that contain homeables and whether or not they are finished
    private PriorityQueue<HomeableCANSparkMax> homeables = new PriorityQueue<>(HomeableCANSparkMax.ORDER_COMPARATOR);
    private ArrayList<HomeableCANSparkMax> activeHoming = new ArrayList<HomeableCANSparkMax>();

    private final HomeableCANSparkMax[] homeableArray;

    public Home(HomeableCANSparkMax... toHome) {
        homeableArray = toHome;
        for (HomeableCANSparkMax homeable : toHome) {
            addRequirements(homeable.getSubsystemObject());
        }
    }

    public Home(HomeableSubsystem subsystem) {
        this(subsystem.getHomeables());
    }

    @Override
    public void initialize() {
        homeables.clear();
        for (HomeableCANSparkMax homeable : homeableArray) {
            homeables.add(homeable);
        }
        activeHoming.clear();
        for (HomeableCANSparkMax homeable : homeables) {
            homeable.updateCurrent();
        }
    }

    @Override
    public void execute() {

        // remove homeables from activeHoming as they finish
        for (HomeableCANSparkMax homeable : new ArrayList<>(activeHoming)) {
            homeable.updateStopCondition();
            if (homeable.isFinished()) {
                activeHoming.remove(homeable);
                homeable.end();
            }
        }

        if (activeHoming.isEmpty() && !homeables.isEmpty()) {
            // fill activeHoming with homeables of the same priority
            while (
                !homeables.isEmpty() && (activeHoming.isEmpty()
                || homeables.peek().getOrder() == activeHoming.get(0).getOrder())
            ) {
                HomeableCANSparkMax homeable = homeables.poll();
                activeHoming.add(homeable);
                homeable.init();
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

    public void end(boolean interrupted) {
        for (HomeableCANSparkMax homeable : activeHoming) {
            homeable.end();
        }
    }
}
