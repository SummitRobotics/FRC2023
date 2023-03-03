package frc.robot.devices;

import edu.wpi.first.util.sendable.Sendable;

/**
 * Interface for Lidar sensors on the robot.
 * There are both LidarV3 and V4 on the robot.
 */
public interface Lidar extends Sendable{
    int getDistance();

    int getAverageDistance();

    /**
     * Compensated the lidar distance for the lidar mount angle in inches.
     *
     * @deprecated I think that this is wrong.
     * @param reportedDistance the distance reported by the lidar
     * @return the new corrected distance
     */
    default double getCompensatedLidarDistance(double reportedDistance, double mountAngle) {
        return reportedDistance * Math.cos(Math.toDegrees(mountAngle)) * 0.393701;
    }
}
