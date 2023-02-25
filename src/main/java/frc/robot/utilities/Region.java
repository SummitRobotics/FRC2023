package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation3d;

public class Region {
    private final Translation3d firstCorner;
    private final Translation3d secondCorner;
    private final Translation3d center;

    /**
     * Creates a region with two corners and a center
     * 
     * @param firstCorner  The first corner of the region. Must have a smaller x, y, and z value than the second corner
     * @param secondCorner The second corner of the region. Must have a larger x, y, and z value than the first corner
     */
    public Region(Translation3d firstCorner, Translation3d secondCorner) {
        this.firstCorner = firstCorner;
        this.secondCorner = secondCorner;
        this.center = new Translation3d((firstCorner.getX() + secondCorner.getX()) / 2, (firstCorner.getY() + secondCorner.getY()) / 2, (firstCorner.getZ() + secondCorner.getZ()) / 2);
    }

    public Translation3d getCenter() {
        return center;
    }

    public boolean contains(Translation3d point) {
        return point.getX() >= firstCorner.getX() && point.getX() <= secondCorner.getX() && point.getY() >= firstCorner.getY() && point.getY() <= secondCorner.getY()
                && point.getZ() >= firstCorner.getZ() && point.getZ() <= secondCorner.getZ();
    }

    public double distanceToOther(Region other) {
        return center.getDistance(other.getCenter());
    }

    public double distanceToPoint(Translation3d point) {
        return center.getDistance(point);
    }

    @Override
    public String toString() {
        return "First Corner: " + firstCorner.toString() + " | Second Corner: " + secondCorner.toString();
    }
}
