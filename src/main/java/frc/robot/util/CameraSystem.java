package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.io.IOException;
import java.util.ArrayList;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;

public class CameraSystem {
    public final AprilTagFieldLayout fieldLayout;
    public final CameraFeed[] cameras;

    public CameraSystem(String[] cameraNames, Transform3d[] robotToCam, String fileName, Field2d field) {
        try {
            fieldLayout = new AprilTagFieldLayout(Filesystem
                    .getDeployDirectory()
                    .toPath()
                    .resolve(fileName));
        } catch (IOException e) {
            e.printStackTrace();
            throw new Error(fileName + " does not exists in the deploy directory");
        }

        cameras = new CameraFeed[cameraNames.length];
        for (int i = 0; i < cameraNames.length; ++i) {
            cameras[i] = new CameraFeed(cameraNames[i], robotToCam[i], fieldLayout);
        }

        field.getObject("April Tag").setPoses(
                fieldLayout.getTags().stream().map(
                        (tag) -> tag.pose.toPose2d()).collect(Collectors.toList()));
    }

    public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d previousePose) {
        ArrayList<EstimatedRobotPose> ret = new ArrayList<EstimatedRobotPose>();

        for (int i = 0; i < cameras.length; ++i) {
            EstimatedRobotPose pose = cameras[i].getEstimatedPose(previousePose);

            if (pose != null) {
                ret.add(pose);
            }
        }

        return ret;
    }
}
