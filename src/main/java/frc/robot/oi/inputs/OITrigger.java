package frc.robot.oi.inputs;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OITrigger {

    private final Map<PrioritizedTrigger, Integer> uses = new HashMap<>();
    private int highestPriority = 0;
    private BooleanSupplier condition;

    public OITrigger(BooleanSupplier condition) {
        this.condition = condition;
    }

    public OITrigger() {
        this.condition = () -> false;
    }

    public PrioritizedTrigger prioritize(int priority) {
        return new PrioritizedTrigger(Math.max(priority, 0));
    }

    public class PrioritizedTrigger {
        private final int priority;

        private PrioritizedTrigger(int priority) {
            this.priority = priority;

            uses.put(this, priority);
            if (priority > highestPriority) {
                highestPriority = priority;
            }
        }

        public Trigger getTrigger() {
            return new Trigger(condition).and(() -> priority >= highestPriority);
        }

        public boolean isValueReal() {
            return priority >= highestPriority;
        }

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
