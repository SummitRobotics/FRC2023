package frc.robot.devices;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utilities.math.RigidTransform3;
import frc.robot.utilities.math.Vector3;

/**
 * Device driver for the limelight.
 */
public class Lemonlight implements Sendable {

    /**
     * Enum for the Type of Limelight software.
     */
    public enum Type {
        Limelight,
        PhotonVision
    }

    private final NetworkTableEntry tv, tx, ty, ta;
    private NetworkTableEntry camMode, pipeline, ledMode;
    private final Type type;
    private final RigidTransform3 position;
    private final double targetHeight;

    /**
     * Creates a new limelight object.
     *
     * @param tableName The table of the limelight. Default is "limelight"
     * @param type The type of camera
     * @param position The position of the limelight. Foward is Y, Right is X and Up is Z.
     * @param targetHeight The height from the camera to the target.
     * @apiNote For both the position and direction vector the positive y-axis is forward on the robot.
     */
    public Lemonlight(String tableName, Type type, RigidTransform3 position, double targetHeight) {
        this.position = position;
        this.targetHeight = targetHeight;
        NetworkTable limelight;
        if (type == Type.Limelight) {
            limelight = NetworkTableInstance.getDefault().getTable(tableName);
            tv = limelight.getEntry("tv");
            tx = limelight.getEntry("tx");
            ty = limelight.getEntry("ty");
            ta = limelight.getEntry("ta");
        } else if (type == Type.PhotonVision) {
            limelight = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(tableName);
            tv = limelight.getEntry("hasTarget");
            tx = limelight.getEntry("targetYaw");
            ty = limelight.getEntry("targetPitch");
            ta = limelight.getEntry("targetArea");
        } else {
            throw new Error("Type must be Limelight or PhotonVision");
        }
        this.type = type;

        //ledMode = limelight.getEntry("ledMode");
        //camMode = limelight.getEntry("camMode");
        //pipeline = limelight.getEntry("pipeline");
    }

    /**
     * Enum to describe the state of the LED.
     */
    public enum LEDModes {
        PIPELINE(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3);

        public final int value;

        LEDModes(int value) {
            this.value = value;
        }
    }

    /**
     * Enum to describe the state of the limelight camera.
     */
    public enum CamModes {
        VISION_PROCESSOR(0),
        DRIVER_CAMERA(1);

        public final int value;

        CamModes(int value) {
            this.value = value;
        }
    }

    /**
     * Sets the LED mode.
     *
     * @param mode the LED mode to switch to
     */
    public void setLEDMode(LEDModes mode) {
        //ledMode.setDouble(mode.value);
    }

    /**
     * Sets the camera mode.
     *
     * @param mode the new mode
     */
    public void setCamMode(CamModes mode) {
        //camMode.setDouble(mode.value);
    }

    /**
     * Sets the pipeline.
     *
     * @param pipe sets the pipeline to an int between 0 and 9
     */
    public void setPipeline(int pipe) {
        //pipeline.setDouble(pipe);
    }

    /**
     * Checks to see if the limelight has a target.
     *
     * @return if limelight has a target
     */
    public boolean hasTarget() {
        if (type == Type.Limelight) {
            return tv.getDouble(0) == 1;
        }
        return tv.getBoolean(false);
    }

    /**
     * Gets the horizontal offset from the limelight.
     *
     * @return the horizontal offset
     */
    public double getHorizontalOffsetRaw() {
        return tx.getDouble(0);
    }

    public double getHorizontalOffset(double distanceOffset, double horizontalOffset) {
        Vector3 targetVector = getTargetVector(distanceOffset, horizontalOffset);
        return Math.atan(targetVector.x / targetVector.y);
    }

    public double getHorizontalOffset() {
        return getHorizontalOffset(0, 0);
    }

    public double getLimelightDistanceEstimate() {
        Vector3 targetVector = getTargetVector();
        return targetVector.add(0, 0, -targetVector.z).length;
    }

    /**
     * Gets the vertical offset from the limelight.
     *
     * @return the vertical offset
     */
    public double getVerticalOffsetRaw() {
        return ty.getDouble(0);
    }

    public double getVerticalOffset(double distanceOffset, double horizontalOffset) {
        Vector3 targetVector = getTargetVector(distanceOffset, horizontalOffset);
        return Math.atan(targetVector.z / targetVector.y);
    }

    public double getVerticalOffset() {
        return getVerticalOffset(0,0);
    }

    /**
     * Gets the percentage of coverage area from the limelight.
     *
     * @return the percentage of area
     */
    public double getAreaPercentage() {
        return ta.getDouble(0);
    }

    private Vector3 getRawTargetVectorDirection() {
        double alpha = Math.toRadians(getHorizontalOffsetRaw());
        double beta = Math.toRadians(getVerticalOffsetRaw());

        // Forward is Y, Right is X, and up is Z
        double y = 1;
        double x = Math.tan(alpha);
        double z = Math.tan(beta);

        Vector3 targetVector = new Vector3(x, y, z);

        return targetVector.rotate(position.rotation.inverse());
    }

    /**
     * THIS IS PROBABLY NOT WHAT YOU WANT. USE getTargetVector INSTEAD.
     * Returns the raw vector from the camera to the target.
     * This is corrected for the cameras angle but not the position.
     *
     * @return The RAW vector from the camera to the target.
     */
    public Vector3 getRawTargetVector() {
        Vector3 vec = getRawTargetVectorDirection();
        return vec.scale(targetHeight / vec.z);
    }

    /**
     * Gets a target vector from the robot to the target with given offsets.
     * The final vector is adjusted for the limelights position and direction.
     *
     * @param distanceOffset The final distance offset.
     * @param horizontalOffset The final horizontal offset.
     * @return A target vector
     */
    public Vector3 getTargetVector(double distanceOffset, double horizontalOffset) {
        Vector3 finalVector = position.translation.add(getRawTargetVector());
        Vector3 distanceOffsetVector = finalVector.add(0, 0, -finalVector.z).norm();
        Vector3 horizontalOffsetVector = distanceOffsetVector.cross(new Vector3(0, 0, 1)).norm();

        return finalVector.add(distanceOffsetVector.scale(distanceOffset)).add(horizontalOffsetVector.scale(horizontalOffset));
    }

    /**
     * Gets a target vector from the robot to the target with default offsets of 0 and 0 for distance and horizontal.
     * The final vector is adjusted for the limelights position and direction.
     * @return A target vector
     */
    public Vector3 getTargetVector() {
        return getTargetVector(0, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Lemonlight");
        //builder.addDoubleProperty("verticalOffset", this::getVerticalOffset, null);
        //builder.addDoubleProperty("horizontalOffset", this::getHorizontalOffset, null);

        builder.addBooleanProperty("hasTarget", this::hasTarget, null);

        builder.addDoubleProperty("offset", this::getHorizontalOffset, null);
        builder.addStringProperty("targetVector", () -> getTargetVector().toString(), null);
        builder.addDoubleProperty("vectorMagnitude", ()-> getTargetVector().length, null);
        builder.addDoubleProperty("targetDistance", () -> getTargetVector().add(0, 0, -getTargetVector().z).length, null);
        //builder.addDoubleProperty("angleVector", ()-> Math.toDegrees(getTargetVector().getAngleInPlane(Vector3D.Plane.XYPlane, Vector3D.Axis.YAxis)), null);
    }
}
