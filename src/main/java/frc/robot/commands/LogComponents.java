package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.utilities.Loggable;

public class LogComponents extends CommandBase {

    ArrayList<BooleanLogEntry> booleanLogs;
    ArrayList<DoubleLogEntry> doubleLogs;
    ArrayList<StringLogEntry> stringLogs;
    DataLog dataLog;
    
    public LogComponents(Loggable... toLog) {

        dataLog = DataLogManager.getLog();

        for (Loggable loggable : toLog) {
            for (HashMap.Entry<String, BooleanSupplier> set
                : loggable.getBooleanLogData().entrySet()) {

                booleanLogs.add(new BooleanLogEntry(dataLog, set.getKey()));
            }

            for (HashMap.Entry<String, DoubleSupplier> set
                : loggable.getDoubleLogData().entrySet()) {
                    
                doubleLogs.add(new DoubleLogEntry(dataLog, set.getKey()));
            }

            for (HashMap.Entry<String, Supplier<String>> set
                : loggable.getStringLogData().entrySet()) {
                    
                stringLogs.add(new StringLogEntry(dataLog, set.getKey()));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
