package frc.robot.commands;

import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utilities.homing.HomeableCANSparkMax;
import frc.robot.utilities.homing.HomeableSubsystem;
import frc.robot.utilities.homing.HomeableCANSparkMax.Type;

public class Home extends CommandBase {

    // maps priorities to maps that contain homeables and whether or not they are finished
    private PriorityQueue<HomeableCANSparkMax> homeables = new PriorityQueue<>(HomeableCANSparkMax.ORDER_COMPARATOR);
    private ArrayList<HomeableCANSparkMax> activeHoming = new ArrayList<HomeableCANSparkMax>();

    private final HomeableCANSparkMax[] homeableArray;

    private final BooleanSupplier forceAtHome;

    public Home(HomeableCANSparkMax... toHome) {
        homeableArray = toHome;
        for (HomeableCANSparkMax homeable : toHome) {
            addRequirements(homeable.getSubsystemObject());
        }
        this.forceAtHome = () -> false;
    }

    public Home(HomeableSubsystem subsystem) {
        this(subsystem.getHomeables());
    }

    public Home(BooleanSupplier forceAtHome, HomeableCANSparkMax... toHome) {
        homeableArray = toHome;
        for (HomeableCANSparkMax homeable : toHome) {
            addRequirements(homeable.getSubsystemObject());
        }
        this.forceAtHome = forceAtHome;
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
        Arm.setDistanceCheck(false);
    }

    @Override
    public void execute() {

        System.out.println(forceAtHome.getAsBoolean());
        // remove homeables from activeHoming as they finish
        for (HomeableCANSparkMax homeable : new ArrayList<>(activeHoming)) {
            homeable.updateStopCondition();
            if (homeable.isFinished() || forceAtHome.getAsBoolean()) {
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
            if (homeable.isFinished() || forceAtHome.getAsBoolean()) homeable.setPower(0); else homeable.setPower();
        }
    }

    public boolean isFinished() {
        return activeHoming.isEmpty() && homeables.isEmpty();
    }

    public void end(boolean interrupted) {
        for (HomeableCANSparkMax homeable : activeHoming) {
            homeable.end();
        }
        Arm.setDistanceCheck(true);
    }
}
