package frc.robot.utilities;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Interface for logging a subsystem or device.
 */
public interface Loggable {

    public String getLogName();

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
