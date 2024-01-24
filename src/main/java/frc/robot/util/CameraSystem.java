package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

public class CameraSystem {
    public final AprilTagFieldLayout fieldLayout;
    public final CameraFeed[] cameras;

    public CameraSystem(String[] cameraNames, Transform3d[] robotToCam, String fileName) {
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
        System.out.println("Cameras" + cameraNames.length);
        for (int i = 0; i < cameraNames.length; ++i) {
            cameras[i] = new CameraFeed(cameraNames[i], robotToCam[i], fieldLayout);
        }
    }

    public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d previousePose) {
        ArrayList<EstimatedRobotPose> ret = new ArrayList<EstimatedRobotPose>();

        for (int i = 0; i < cameras.length; ++i) {
            EstimatedRobotPose pose = cameras[i].getEstimatedPose(previousePose);

            System.out.println("Pose: " + pose);

            if (pose != null) {
                ret.add(pose);
            }
        }

        return ret;
    }
}
