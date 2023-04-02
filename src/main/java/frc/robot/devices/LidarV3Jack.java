package frc.robot.devices;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.TimerTask;

/**
 * Driver for a LidarLite sensor
 */
public class LidarV3Jack implements Lidar {
    private I2C mI2C;
    private byte[] mDistance;
    private java.util.Timer mUpdater;
    private boolean mHasSignal;
    private int count = 0;
    
    private final static int LIDAR_ADDR = 0x62;
    private final static int LIDAR_CONFIG_REGISTER = 0x00;
    private final static int LIDAR_DISTANCE_REGISTER = 0x8f;
    //private final static int LIDAR_DEFAULT_ADDRESS = 0x1E;
    private final int mVersion;

    public LidarV3Jack(Port port) {
    	this(port, 2);
    }

    public LidarV3Jack(Port port, int version) {
        mI2C = new I2C(port, LIDAR_ADDR);
        mDistance = new byte[2];
        mUpdater = new java.util.Timer();
        mHasSignal = false;
        mVersion = (version != 2 && version != 3) ? 2 : version;
        start();
    }

    /**
     * @return Distance in centimeters
     */
    @Override

    public int getDistance() {
        int distCm = (int) Integer.toUnsignedLong(mDistance[0] << 8) + Byte.toUnsignedInt(mDistance[1]);
        return distCm;
    }

    /**
     * @return true iff the sensor successfully provided data last loop
     */
    public boolean hasSignal() {
        return mHasSignal;
    }

    /**
     * Start 10Hz polling
     */
    public void start() {
        start(100);
    }

    /**
     * Start polling for period in milliseconds
     */
    public void start(int period) {
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
            	update();
            }
        };

        mUpdater.scheduleAtFixedRate(task, 0, period);
    }

    public void stop() {
        mUpdater.cancel();
        mUpdater = new java.util.Timer();
    }

    private void update()
    {
        if (mVersion == 3) {
        	updateV3();
        }
        else { 
        	updateV2();
        }
    }
    
    private void updateV2() {
        // Transfer aborted: true = aborted; false = success
    	if (mI2C.write(LIDAR_CONFIG_REGISTER, 0x04)) {
            // the write failed to ack
            mHasSignal = false;
            //return;
        }
        
        Timer.delay(0.04); // Delay for measurement to be taken
        // Transfer aborted: true = aborted; false = success
        if (mI2C.read(LIDAR_DISTANCE_REGISTER, 2, mDistance)) {
            // the read failed
            mHasSignal = false;
            //return;
        }

        int distCm = (int) Integer.toUnsignedLong(mDistance[0] << 8) + Byte.toUnsignedInt(mDistance[1]);
        SmartDashboard.putNumber("Raw Lidar", distCm);
        SmartDashboard.putNumber("Lidar Sample COunt", count++);
        mHasSignal = true;
        Timer.delay(0.005); // Delay to prevent over polling
    }
    
    private void updateV3() {
    	mI2C.write(LIDAR_CONFIG_REGISTER, 0x04);
        Timer.delay(0.005); // Delay to prevent over polling
    	
        byte [] sendData = new byte [1];
        sendData[0] = (byte)0x01;
        mI2C.writeBulk(sendData, 1);
        mI2C.readOnly(mDistance, 2);
        Timer.delay(0.001); // Delay to prevent over polling
   
        sendData[0] = (byte)0x8F;
        mI2C.writeBulk(sendData, 1);
        mI2C.readOnly(mDistance, 2);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public int getAverageDistance() {
        return getDistance();
    }
}