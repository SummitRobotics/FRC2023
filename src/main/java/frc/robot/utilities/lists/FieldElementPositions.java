package frc.robot.utilities.lists;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.arm.ArmConfiguration;
import frc.robot.subsystems.arm.ArmConfiguration.POSITION_TYPE;

public final class FieldElementPositions {

    public static final ArmConfiguration
        substationPregrab = new ArmConfiguration(93.00, 7.00, 114, -50.16, -51.80, POSITION_TYPE.ENCODER_ROTATIONS);

    // values of game elements placed at double substations
    public static final Translation3d
        BLUE_LEFT_SUBSTATION = new Translation3d(16.273, 7.545, 1.07),
        BLUE_RIGHT_SUBSTATION = new Translation3d(16.273, 6.092, 1.07),
        RED_LEFT_SUBSTATION = new Translation3d(0.274, 6.092, 1.07),
        RED_RIGHT_SUBSTATION = new Translation3d(0.274, 7.545, 1.07);

    // points to drive to for accessing double substation
    public static final Pose2d
        BLUE_LEFT_DRIVE_POINT_PRE = new Pose2d(
            BLUE_LEFT_SUBSTATION.getX() - 1.6,
            BLUE_LEFT_SUBSTATION.getY(),
            new Rotation2d(4, 0)
        ),
        BLUE_RIGHT_DRIVE_POINT_PRE = new Pose2d(
            BLUE_RIGHT_SUBSTATION.getX() - 1.6,
            BLUE_RIGHT_SUBSTATION.getY(),
            new Rotation2d(4, 0)
        ),
        RED_LEFT_DRIVE_POINT_PRE = new Pose2d(
            RED_LEFT_SUBSTATION.getX() + 1.6,
            RED_LEFT_SUBSTATION.getY(),
            new Rotation2d(-4, 0)
        ),
        RED_RIGHT_DRIVE_POINT_PRE = new Pose2d(
            RED_RIGHT_SUBSTATION.getX() + 1.6,
            RED_RIGHT_SUBSTATION.getY(),
            new Rotation2d(-4, 0)
        ),
        BLUE_LEFT_DRIVE_POINT = new Pose2d(
            BLUE_LEFT_SUBSTATION.getX() - 1.2,
            BLUE_LEFT_SUBSTATION.getY(),
            new Rotation2d(4, 0)
        ),
        BLUE_RIGHT_DRIVE_POINT = new Pose2d(
            BLUE_RIGHT_SUBSTATION.getX() - 1.2,
            BLUE_RIGHT_SUBSTATION.getY(),
            new Rotation2d(4, 0)
        ),
        RED_LEFT_DRIVE_POINT = new Pose2d(
            RED_LEFT_SUBSTATION.getX() + 1.2,
            RED_LEFT_SUBSTATION.getY(),
            new Rotation2d(-4, 0)
        ),
        RED_RIGHT_DRIVE_POINT = new Pose2d(
            RED_RIGHT_SUBSTATION.getX() + 1.2,
            RED_RIGHT_SUBSTATION.getY(),
            new Rotation2d(-4, 0)
        );
    
    public static final double[]
        // Arrays to calculate coordinates of nodes.
        // Ordering is important here to correctly convert from node id to coordinates.
        // Blue x and y coordinates increase from low to high numbering, red x and y decrease.
        // Z coordinates increase with numbering for blue, decrease for red.
        BLUE_X_VALUES = {1.17, 0.816, 0.371},
        BLUE_Y_VALUES = {4.986, 4.417, 3.870, 3.295, 2.737, 2.184, 1.615, 1.041, 0.40},
        BLUE_Z_VALUES = {0.3, 1.1414, 1.35},
        RED_X_VALUES = {15.395, 15.737, 16.149},
        RED_Y_VALUES = {0.40, 1.041, 1.615, 2.184, 2.737, 3.295, 3.870, 4.417, 4.986},
        RED_Z_VALUES = {0.3, 1.1414, 1.35};
}
