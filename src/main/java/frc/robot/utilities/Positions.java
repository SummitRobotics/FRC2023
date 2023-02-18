package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Drivetrain;

public class Positions {
    public static class Pose3d {

        enum Space {
            ROBOT, FIELD
        }

        // In Robot Space
        private final edu.wpi.first.math.geometry.Pose3d pose;
        private final Space space;

        private Pose3d(edu.wpi.first.math.geometry.Pose3d pose, Space space) {
            this.pose = pose;
            this.space = space;
        }
        
        public static Pose3d fromRobotSpace(edu.wpi.first.math.geometry.Pose3d pose) {
            return new Pose3d(pose, Space.ROBOT);
        }

        public static Pose3d fromFieldSpace(edu.wpi.first.math.geometry.Pose3d pose) {
            return new Pose3d(pose, Space.FIELD);
        }

        public static Pose3d fromOtherSpace(edu.wpi.first.math.geometry.Pose3d pose, Transform3d robotToOther) {
            Transform3d otherToRobot = robotToOther.inverse();
            return new Pose3d(pose.plus(otherToRobot), Space.ROBOT);
        }

        public static Pose3d fromRobotSpace(Translation3d pose) {
            return fromRobotSpace(new edu.wpi.first.math.geometry.Pose3d(pose, new Rotation3d()));
        }

        public static Pose3d fromFieldSpace(Translation3d pose) {
            return fromFieldSpace(new edu.wpi.first.math.geometry.Pose3d(pose, new Rotation3d()));
        }

        public static Pose3d fromOtherSpace(Translation3d pose, Transform3d robotToOther) {
            return fromOtherSpace(new edu.wpi.first.math.geometry.Pose3d(pose, new Rotation3d()), robotToOther);
        }

        public edu.wpi.first.math.geometry.Pose3d inRobotSpace() {
            if (space == Space.ROBOT) {
                return pose;
            } else {
                return pose.relativeTo(new edu.wpi.first.math.geometry.Pose3d(Drivetrain.getInstance().getPose()));
            }
        }

        public edu.wpi.first.math.geometry.Pose3d inFieldSpace() {
            if (space == Space.FIELD) {
                return pose;
            } else {
                edu.wpi.first.math.geometry.Pose3d robotPose = new edu.wpi.first.math.geometry.Pose3d(Drivetrain.getInstance().getPose());
                return pose.plus(new Transform3d(robotPose.getTranslation(), robotPose.getRotation()));
            }
        }

        public edu.wpi.first.math.geometry.Pose3d inOtherSpace(Transform3d robotToOther) {
            if (space == Space.ROBOT) {
                return pose.plus(robotToOther);
            } else {
                edu.wpi.first.math.geometry.Pose3d robotPose = new edu.wpi.first.math.geometry.Pose3d(Drivetrain.getInstance().getPose());
                return pose.relativeTo(robotPose).plus(robotToOther);
            }
        }
    }
}