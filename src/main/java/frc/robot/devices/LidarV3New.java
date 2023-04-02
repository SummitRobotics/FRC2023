package frc.robot.devices;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utilities.RollingAverage;

/**
 * Class for using the lidar v4.
 */
public class LidarV3New implements Lidar {

    @SuppressWarnings("CheckStyle")
    private final I2C portI2C;
    private int value;

    private int lidarReadCount;
    private int lidarWriteCount;
    private int lidarReadErrorCount;
    private int lidarWriteErrorCount;

    private final RollingAverage rollingAverage;

    private final Runnable proximityReader;
    private Notifier thread;
    private Object valueLock;
    private Object loopLock;
    private Timer loopTimer;
    private double measuredLoopTime;

    /**
     * constructor.
     *
     * @param id the i2c id of the lidarV4
     */
    public LidarV3New(int id) {
        portI2C = new I2C(Port.kMXP, id);
        value = 0;
        lidarReadCount = 0;
        lidarWriteCount = 0;
        lidarReadErrorCount = 0;
        lidarWriteErrorCount = 0;
        measuredLoopTime = 100;

        // TODO CHANGE THIS
        rollingAverage = new RollingAverage(1, true);

        valueLock = new Object();
        loopLock = new Object();
        loopTimer = new Timer();
        proximityReader = new Runnable() {
            @Override
            public void run() {
                loopTimer.start();
                int distance = readDistance();
                if (distance != -1) {
                    updateValue(distance);
                }
                loopTimer.stop();
                updateLoopTime(loopTimer.get());
                loopTimer.reset();
            }
        };

        startMeasuring();
        thread = new Notifier(proximityReader);
        thread.startPeriodic(0.02);
    }

    public LidarV3New() {
        this(0x62);
        SmartDashboard.putNumber("LidarReadCount", lidarReadCount);
        SmartDashboard.putNumber("LidarWriteCount", lidarWriteCount);
        SmartDashboard.putNumber("LidarReadErrorCount", lidarReadErrorCount);
        SmartDashboard.putNumber("LidarWriteErrorCount", lidarWriteErrorCount);
    }

    private void write(int registerAddress, int data) {
        byte[] buffer = new byte[2];
        buffer[0] = (byte) registerAddress;
        buffer[1] = (byte) data;
        int result = I2CJNI.i2CWriteB(portI2C.getPort(), (byte)portI2C.getDeviceAddress(), buffer, (byte)buffer.length);
        if (result < 0) {
            if (lidarWriteCount % 10 == 0) {
                System.out.println("Aborted register write: " + result);
            }
            SmartDashboard.putNumber("LidarWriteErrorCount", ++lidarWriteErrorCount);
        } else {
            SmartDashboard.putNumber("LidarWriteCount", ++lidarWriteCount); 
        }
    }

    private void read(int registerAddress, int count, byte[] buffer) {
        if (portI2C.read(registerAddress, count, buffer)) {
            if (lidarReadErrorCount % 10 == 0) {
                System.out.println("Aborted register read: " + registerAddress);
            }
            SmartDashboard.putNumber("LidarReadErrorCount", ++lidarReadErrorCount);
        } else {
            SmartDashboard.putNumber("LidarReadCount", ++lidarReadCount);   
        }
    }
    private void startMeasuring() {
        // Reset power state
        write(0x65, 0x80); // TRY THIS MAYBE IF NOT WORKING

        // Use delay from MEASURE_DELAY (0x45) for burst and free running modes
        write(0x04, 0b100000); // This Allows For Manual Setting Of Delay

        // Delay between automatic measurements (0x14 default)
        write(0x45, 0x14); // Delay between measurements

        // Burst measurement count control (0x01 default)
        write(0x11, 0x01); // Mght want to try 0x00 if not working TRY THIS FIRST
    }

    /**
     * reads the current distance from the lidar if one is available.
     */
    private int readDistance() {
        byte[] status = new byte[1];

        // Take distance measurement with receiver bias correction
        write(0x00, 0x04);

        // checks if there is a valid measurement
        read(0x01, 1, status);

        // Device is not busy taking measurement
        if ((status[0] & 1) == 0) {
            byte[] low = new byte[1];
            byte[] high = new byte[1];  

            // reads distance from lidar (0x0f is high byte, 0x10 is low byte)
            read(0x0f, 1, high);
            read(0x10, 1, low);
            
            int out = (high[0] << 8) + low[0];

            // int out = low[0] & 0xff + ((high[0] & 0xff) << 8);
            //System.out.println("LidarV3New RAW (IN LIDARV3New.java): " + out);

            return out;
        }
        return -1;
    }

    /**
     * Gets the most recent distance.
     *
     * @return the distance in cm
     */
    @Override
    public int getDistance() {
        synchronized (valueLock) {
            return value;
        }
    }

    /**
     * Returns how long measurements took.
     *
     * @return The amount of time, in milliseconds, that it took to measure distance
     */
    public double getLoopTimeMilliseconds() {
        synchronized (loopLock) {
            return measuredLoopTime * 1000;
        }
    }

    private void updateValue(int value) {
        if (value != -1) {
            synchronized (valueLock) {
                this.value = value;
                rollingAverage.update(value);
            }
        }
    }

    private void updateLoopTime(double time) {
        synchronized (loopLock) {
            measuredLoopTime = time;
        }
    }

    /**
     * Gets the average distance.
     *
     * @return average distance in cm
     */
    @Override
    public int getAverageDistance() {
        synchronized (valueLock) {
            return (int) rollingAverage.getAverage();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("LidarV3New");
        builder.addDoubleProperty("avgDistance", this::getAverageDistance, null);
        builder.addDoubleProperty("distance", this::getDistance, null);
    }
}
