package frc.robot.commands;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.utilities.Loggable;

/**
 * Logs data for any number of Loggable components
 */
public class LogComponents extends CommandBase {

    Loggable[] toLog;
    HashMap<String, BooleanLogEntry> booleanEntries;
    HashMap<String, DoubleLogEntry> doubleEntries;
    HashMap<String, StringLogEntry> stringEntries;
    DataLog dataLog;
    
    /*
     * Logs data for any number of Loggable components.
     */
    public LogComponents(Loggable... toLog) {

        this.toLog = toLog;
        dataLog = DataLogManager.getLog();

        // Iterates through the supplier HashMaps of each Loggable object and generates entries.
        for (Loggable loggable : toLog) {
            for (HashMap.Entry<String, BooleanSupplier> set
                : loggable.getBooleanLogData().entrySet()) {

                String name = loggable.getLogName() + "/" + set.getKey();
                BooleanLogEntry entry = new BooleanLogEntry(dataLog, name);

                entry.setMetadata(name);
                booleanEntries.put(name, entry);
            }

            for (HashMap.Entry<String, DoubleSupplier> set
                : loggable.getDoubleLogData().entrySet()) {

                String name = loggable.getLogName() + "/" + set.getKey();
                DoubleLogEntry entry = new DoubleLogEntry(dataLog, name);
                
                entry.setMetadata(name);
                doubleEntries.put(name, entry);
            }

            for (HashMap.Entry<String, Supplier<String>> set
                : loggable.getStringLogData().entrySet()) {

                String name = loggable.getLogName() + "/" + set.getKey();
                StringLogEntry entry = new StringLogEntry(dataLog, name);
                
                entry.setMetadata(name);
                stringEntries.put(name, entry);
            }
        }
    }

    @Override
    public void initialize() {
        DataLogManager.start();
    }

    @Override
    public void execute() {

        // Iterates through the supplier HashMaps of each Loggable object and appends the
        // latest data records to the entries.
        for (Loggable loggable : toLog) {
            for (HashMap.Entry<String, BooleanSupplier> set
                : loggable.getBooleanLogData().entrySet()) {

                booleanEntries.get(loggable.getLogName() + "/" + set.getKey())
                    .append(set.getValue().getAsBoolean());
            }

            for (HashMap.Entry<String, DoubleSupplier> set
                : loggable.getDoubleLogData().entrySet()) {

                doubleEntries.get(loggable.getLogName() + "/" + set.getKey())
                    .append(set.getValue().getAsDouble());
            }

            for (HashMap.Entry<String, Supplier<String>> set
                : loggable.getStringLogData().entrySet()) {

                stringEntries.get(loggable.getLogName() + "/" + set.getKey())
                    .append(set.getValue().get());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
