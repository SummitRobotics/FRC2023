package frc.robot.utilities;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Interface for logging a subsystem or device.
 */
public interface Loggable {

    // Name of the entire Loggable object (e.g. drivetrain, lidar).
    public String getLogName();

    // Each key is the name of an individual data source; each value is a supplier for that data.
    // All default to an empty HashMap.

    public default HashMap<String, BooleanSupplier> getBooleanLogData() {
        return new HashMap<String, BooleanSupplier>();
    }

    public default HashMap<String, DoubleSupplier> getDoubleLogData() {
        return new HashMap<String, DoubleSupplier>();
    }

    public default HashMap<String, Supplier<String>> getStringLogData() {
        return new HashMap<String, Supplier<String>>();
    }
}
