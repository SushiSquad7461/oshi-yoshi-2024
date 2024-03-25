package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraFeed {
    public final PhotonCamera camera;

    private PhotonPoseEstimator photonPoseEstimator;
 
    public CameraFeed(String name, Transform3d robotToCam, AprilTagFieldLayout aprilTagFieldLayout) {
        camera = new PhotonCamera(name);

        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            camera, 
            robotToCam
        ); // Change later to process on orange pi
    }

    public EstimatedRobotPose getEstimatedPose(Pose2d previsouePose) {
        photonPoseEstimator.setReferencePose(previsouePose);
 
        Optional<EstimatedRobotPose> res = photonPoseEstimator.update();

        if (res.isEmpty()) {
            return null;
        }

        return res.get();
    }
}
