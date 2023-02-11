package frc.robot.devices;

/**
 * Interface for Lidar sensors on the robot.
 * There are both LidarV3 and V4 on the robot.
 */
public interface Lidar {
    int getDistance();

    int getAverageDistance();

    double getLoopTimeMilliseconds();

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
