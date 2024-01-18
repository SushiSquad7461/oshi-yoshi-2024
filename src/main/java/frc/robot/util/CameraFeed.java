package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraFeed {
    public final PhotonCamera camera;

    private final NetworkTableEntry rawBytesEntry;
    private double lastUpdateTimeMicro;

    private PhotonPoseEstimator photonPoseEstimator;

    private ArrayList<VisionMeasurement> cachedMesurments;
    private AprilTagFieldLayout aprilTagFieldLayout;

    public CameraFeed(String name, Transform3d robotToCam, AprilTagFieldLayout aprilTagFieldLayout) {
        camera = new PhotonCamera(name);
        
        rawBytesEntry = NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable(name)
            .getEntry("rawBytes");


        this.lastUpdateTimeMicro = -1;
        cachedMesurments = new ArrayList<VisionMeasurement>();
        this.aprilTagFieldLayout = aprilTagFieldLayout;

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, camera, robotToCam); // Change later to process on orange pi
    }

    public ArrayList<VisionMeasurement> getMesurments() {
        if (lastUpdateTimeMicro == rawBytesEntry.getLastChange()) {
            return cachedMesurments;
        } else {
            lastUpdateTimeMicro = rawBytesEntry.getLastChange();
        }

        PhotonPipelineResult res = camera.getLatestResult();

        if (!res.hasTargets()) {
            cachedMesurments.clear();
            return cachedMesurments;
        }


        cachedMesurments.clear();

        for (PhotonTrackedTarget target : res.targets) {
            // Get pose
            Pose3d estRobotPose = getRobotPoseFromTarget(target);
            if (estRobotPose == null) {
                continue;
            }

            // Add to measurement
            cachedMesurments.add(new VisionMeasurement(
                target,
                new Pose2d(
                    estRobotPose.getX(),
                    estRobotPose.getY(),
                    new Rotation2d(estRobotPose.getRotation().getZ())), 
                    res.getTimestampSeconds(),
                target.getPoseAmbiguity())); 
        }

        return cachedMesurments;
    }

    public EstimatedRobotPose getEstimatedPose() {
        Optional<EstimatedRobotPose> res = photonPoseEstimator.update();

        if (res.isEmpty()) {
            return null;
        }

        return res.get(); 
    }


    /**
     * Using an april tag location locate the relative position of the robot.
     */
    public Pose3d getRobotPoseFromTarget(PhotonTrackedTarget target) {
        // Get transform that converts from camera pose to target pose
        Transform3d cameraToTarget = target.getBestCameraToTarget();

        // Get the april tag's pose on the field
        Optional<Pose3d> feducialPos = aprilTagFieldLayout.getTagPose(target.getFiducialId());

        // If we don't have a pose for that apriltag id, then skip
        if (feducialPos.isEmpty()) {
            return null;
        }

        return PhotonUtils.estimateFieldToRobotAprilTag(
            cameraToTarget,
            feducialPos.get(), 
            cameraToTarget
        );
    }
}
