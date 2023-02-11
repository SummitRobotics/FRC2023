package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Positions {
    public static class Pose3d {
        // In Robot Space
        private final edu.wpi.first.math.geometry.Pose3d pose;

        private Pose3d(edu.wpi.first.math.geometry.Pose3d pose) {
            this.pose = pose;
        }
        
        public static Pose3d fromRobotSpace(edu.wpi.first.math.geometry.Pose3d pose) {
            return new Pose3d(pose);
        }

        public static Pose3d fromFieldSpace(edu.wpi.first.math.geometry.Pose3d pose, edu.wpi.first.math.geometry.Pose3d robotPose) {
            Transform3d fieldToRobot = new Transform3d(new edu.wpi.first.math.geometry.Pose3d(), robotPose);
            return new Pose3d(pose.plus(fieldToRobot));
        }

        public static Pose3d fromOtherSpace(edu.wpi.first.math.geometry.Pose3d pose, Transform3d robotToOther) {
            Transform3d otherToRobot = robotToOther.inverse();
            return new Pose3d(pose.plus(otherToRobot));
        }

        public edu.wpi.first.math.geometry.Pose3d inRobotSpace() {
            return pose;
        }

        public edu.wpi.first.math.geometry.Pose3d inFieldSpace(edu.wpi.first.math.geometry.Pose3d robotPose) {
            Transform3d fieldToRobot = new Transform3d(new edu.wpi.first.math.geometry.Pose3d(), robotPose);
            return pose.plus(fieldToRobot.inverse());
        }

        public edu.wpi.first.math.geometry.Pose3d inOtherSpace(Transform3d robotToOther) {
            return pose.transformBy(robotToOther);
        }
    }
}