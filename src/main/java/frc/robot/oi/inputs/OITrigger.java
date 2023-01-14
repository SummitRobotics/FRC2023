package frc.robot.oi.inputs;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for generating triggers that take into account button priorities.
 * 
 * Most commands will be passed an OITrigger for each button and will need to use
 * .prioritize().getTrigger() to get a Trigger instance. Follow the directions at 
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
 * to then bind some kind of response to that trigger.
 * 
 * Example:
 * 
 * OITrigger oitrigger = new OITrigger(() -> true);
 * oitrigger.prioritize(5).getTrigger().onTrue(
 *     new InstantCommand(() -> System.out.println("Test")));
 */
public class OITrigger {

    private final Map<PrioritizedTrigger, Integer> uses = new HashMap<>();
    private int highestPriority = 0;
    private BooleanSupplier condition;

    /**
     * Creates an OITrigger from a condition.
     *
     * @param condition BooleanSupplier for the condition
     */
    public OITrigger(BooleanSupplier condition) {
        this.condition = condition;
    }

    /**
     * Creates an OITrigger that will always be inactive.
     */
    public OITrigger() {
        this.condition = () -> false;
    }

    /**
     * Returns an instance of a PrioritizedTrigger from the OITrigger.
     *
     * @param priority The priority, generally from 0 to 10
     * @return a PrioritizedTrigger
     */
    public PrioritizedTrigger prioritize(int priority) {
        return new PrioritizedTrigger(Math.max(priority, 0));
    }

    /**
     * Inner class for PrioritizedTrigger
     */
    public class PrioritizedTrigger {
        private final int priority;

        /**
         * Constructor for PrioritizedTrigger
         */
        private PrioritizedTrigger(int priority) {
            this.priority = priority;

            uses.put(this, priority);
            if (priority > highestPriority) {
                highestPriority = priority;
            }
        }

        /**
         * Returns an ordinary Trigger that works with the priority system.
         *
         * @return A Trigger instance
         */
        public Trigger getTrigger() {
            return new Trigger(condition).and(() -> priority >= highestPriority);
        }

        /**
         * Returns whether or not this PrioritizedTrigger currently has priority
         * over all other PrioritizedTriggers made by the same OITrigger.
         *
         * @return A boolean determining whether or not the value was a default
         */
        public boolean isValueReal() {
            return priority >= highestPriority;
        }

        /**
         * Removes the prioritized axis from the HashMap of prioritized triggers.
         */
        public void destroy() {
            uses.remove(this);

            if (priority >= highestPriority) {
                int newHighestPriority = 0;
                for (int i : uses.values()) {
                    newHighestPriority = Math.max(i, newHighestPriority);
                }
                highestPriority = newHighestPriority;
            }
        }
    }
}
