/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.devices;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
 
public class AprilTagCameraWrapper {
     public PhotonCamera photonCamera;
     public PhotonPoseEstimator photonPoseEstimator;
 
     public AprilTagCameraWrapper(String cameraName, Transform3d robotToCamera) throws IOException {
         AprilTagFieldLayout atfl =
                 new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
 
         // Camera
         photonCamera =
                 new PhotonCamera(cameraName);
 
         // Create pose estimator
         photonPoseEstimator =
                 new PhotonPoseEstimator(
                         atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, robotToCamera);
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
      *     of the observation. Assumes a planar field and the robot is always firmly on the ground
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
         return photonPoseEstimator.update();
     }

     public double getTargetDistance() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (!result.hasTargets()) {
                return 100;
        }

        return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
     }

     public void forceDisableDriverMode() {
         photonCamera.setDriverMode(false);
     }

     public static class EstimatedRobotPoseWithSD {
        public final EstimatedRobotPose estimatedRobotPose;
        public final double sd;

        public EstimatedRobotPoseWithSD(EstimatedRobotPose estimatedRobotPose, double sd) {
            this.estimatedRobotPose = estimatedRobotPose;
            this.sd = sd;
        }
     }
 }
 