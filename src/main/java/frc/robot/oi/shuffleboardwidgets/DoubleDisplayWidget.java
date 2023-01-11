package frc.robot.oi.shuffleboardwidgets;

import edu.wpi.first.networktables.DoublePublisher;

/**
 * Wrapper for double-write-only Shuffleboard widgets.
 */
public class DoubleDisplayWidget {

    private final DoublePublisher entry;

    /**
     * Displays value on a widget.
     *
     * @param entry the network table entry to display.
     * @apiNote THIS JAVA DOC MIGHT BE WRONG!
     */
    public DoubleDisplayWidget(DoublePublisher entry) {

        this.entry = entry;
        entry.set(0);
    }

    /**
     * Sets the value the display should show.
     *
     * @param value the value that the display should show
     */
    public void setValue(double value) {
        entry.set(value);
    }
}
