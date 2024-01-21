package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;

import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleNeoTalon;
import SushiFrcLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.CameraSystem;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController rotationLockPID;
    private CameraSystem cameraSystem;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

    private Swerve() {
        super(
                new SwerveModuleNeoTalon[] {
                        new SwerveModuleNeoTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[0]),
                        new SwerveModuleNeoTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[1]),
                        new SwerveModuleNeoTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[2]),
                        new SwerveModuleNeoTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[3]),
                },
                new Pigeon(Constants.Ports.PIGEON_ID, Constants.Swerve.GYRO_INVERSION, Constants.Ports.CANIVORE_NAME),
                Constants.Swerve.SWERVE_KINEMATICS);

        locationLock = false;
        rotationLockPID = Constants.Swerve.autoRotate.getPIDController();

        cameraSystem = new CameraSystem(new String[] { "camera1" }, new Transform3d[] { new Transform3d() },
                "apriltags.json");
    }

    public void enableRotationLock(double angle) {
        locationLock = true;

        rotationLockPID.setSetpoint(angle);
        rotationLockPID.calculate(getGyro().getAngle().getDegrees());
    }

    public void disableRotationLock() {
        locationLock = false;
    }

    @Override
    public void drive(Translation2d translation, double rotation) {
        if (locationLock) {
            rotation = rotationLockPID.calculate(getGyro().getAngle().getDegrees());
        }

        drive(translation, rotation);
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    @Override
    public void periodic() {
        super.periodic();

        ArrayList<EstimatedRobotPose> list = cameraSystem.getEstimatedPoses();

        field.getObject("April Tag").setPoses(
                list.stream().map(
                        (estimate) -> estimate.estimatedPose.toPose2d()).collect(Collectors.toList()));
    }
}
