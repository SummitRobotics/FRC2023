package frc.robot.utilities.lists;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class FieldElementPositions {
    // we only use the double substation
    public static final Translation3d
        BLUE_LEFT_GAME_PIECE = new Translation3d(16.38457, 7.32554, 1.07),
        BLUE_RIGHT_GAME_PIECE = new Translation3d(16.38457, 6.21234, 1.07),
        RED_LEFT_GAME_PIECE = new Translation3d(0.15288, 6.13712, 1.07),
        RED_RIGHT_GAME_PIECE = new Translation3d(0.15288, 7.47597, 1.07);
    
    public static final Pose2d
        BLUE_LEFT_DRIVE_POINT = new Pose2d(
            BLUE_LEFT_GAME_PIECE.getX() - 0.6,
            BLUE_LEFT_GAME_PIECE.getY(),
            new Rotation2d(4, 0)
        ),
        BLUE_RIGHT_DRIVE_POINT = new Pose2d(
            BLUE_RIGHT_GAME_PIECE.getX() - 0.6,
            BLUE_RIGHT_GAME_PIECE.getY(),
            new Rotation2d(4, 0)
        ),
        RED_LEFT_DRIVE_POINT = new Pose2d(
            RED_LEFT_GAME_PIECE.getX() + 0.6,
            RED_LEFT_GAME_PIECE.getY(),
            new Rotation2d(-4, 0)
        ),
        RED_RIGHT_DRIVE_POINT = new Pose2d(
            RED_RIGHT_GAME_PIECE.getX() + 0.6,
            RED_RIGHT_GAME_PIECE.getY(),
            new Rotation2d(-4, 0)
        );

    public static final double[]
        // For indices to work properly: lowest to highest x, highest to lowest y for blue
        // highest to lowest x, lowest to highest y for red
        BLUE_X_VALUES = {0.42068, 1.0525, 1.63919, 2.1657, 2.73734, 3.29394, 3.8355, 4.40715, 5.05401},
        BLUE_Y_VALUES = {1.16078, 0.7847, 0.36348},
        RED_X_VALUES = {5.02392, 4.42219, 3.82046, 3.29394, 2.69221, 2.1657, 1.6091, 1.0525, 0.42068},
        RED_Y_VALUES = {15.36162, 15.70762, 16.20405},
        BOTH_Z_VALUES = {0, 1, 1.2954};
}
