package frc.robot.utilities.lists;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class FieldElementPositions {
    // we only use the double substation
    public static final Translation3d BLUE_LEFT_GAME_PIECE 
        = new Translation3d(16.38457, 7.32554, 1.07);
    public static final Translation3d BLUE_RIGHT_GAME_PIECE 
        = new Translation3d(16.38457, 6.21234, 1.07);
    public static final Translation3d RED_LEFT_GAME_PIECE 
        = new Translation3d(0.15288, 6.13712, 1.07);
    public static final Translation3d RED_RIGHT_GAME_PIECE 
        = new Translation3d(0.15288, 7.47597, 1.07);
    public static final Pose2d BLUE_LEFT_DRIVE_POINT
        = new Pose2d(15.858, 7.32554, new Rotation2d(4, 0));
    public static final Pose2d BLUE_RIGHT_DRIVE_POINT
        = new Pose2d(15.858, 6.21234, new Rotation2d(4, 0));
    public static final Pose2d RED_LEFT_DRIVE_POINT
        = new Pose2d(0.815, 6.13712, new Rotation2d(-4, 0));
    public static final Pose2d RED_RIGHT_DRIVE_POINT
        = new Pose2d(0.815, 7.47597, new Rotation2d(-4, 0));
}
