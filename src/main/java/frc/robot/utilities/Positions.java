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
            return new Pose3d(pose.relativeTo(new edu.wpi.first.math.geometry.Pose3d(robotToOther.inverse().getTranslation(), robotToOther.inverse().getRotation())), Space.ROBOT);
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
                // System.out.println("Converting from field space to robot space");
                // System.out.println(pose);
                // System.out.println(Drivetrain.getInstance().getPose());
                return pose.relativeTo(new edu.wpi.first.math.geometry.Pose3d(Drivetrain.getInstance().getPose()));
            }
        }

        public edu.wpi.first.math.geometry.Pose3d inFieldSpace() {
            if (space == Space.FIELD) {
                return pose;
            } else {
                edu.wpi.first.math.geometry.Pose3d robotPose = new edu.wpi.first.math.geometry.Pose3d(Drivetrain.getInstance().getPose());
                Transform3d robotToField = new Transform3d(robotPose.getTranslation(), robotPose.getRotation());
                return pose.relativeTo(new edu.wpi.first.math.geometry.Pose3d(robotToField.inverse().getTranslation(), robotToField.inverse().getRotation()));
            }
        }

        public edu.wpi.first.math.geometry.Pose3d inOtherSpace(Transform3d robotToOther) {
            if (space == Space.ROBOT) {
                return pose.relativeTo(new edu.wpi.first.math.geometry.Pose3d(robotToOther.getTranslation(), robotToOther.getRotation()));
            } else {
                return inRobotSpace().relativeTo(new edu.wpi.first.math.geometry.Pose3d(robotToOther.getTranslation(), robotToOther.getRotation()));
            }
        }

        @Override
        public String toString() {
            if (space == Space.ROBOT) {
                return "Robot Space: " + pose;
            }
            return "Field Space: " + pose;
        }
    }
}