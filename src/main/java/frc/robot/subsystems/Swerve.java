package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.SmartDashboard.TunableNumber;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleTalon;
import SushiFrcLib.Swerve.SwerveTemplates.VisionBaseSwerve;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController rotationLockPID;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator camFilter;
    private final TunableNumber maxDistanceCamToTarget;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

    private Swerve() {
        super(
                new SwerveModuleTalon[] {
                        new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[0]),
                        new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[1]),
                        new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[2]),
                        new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[3]),
                },
                new Pigeon(Constants.Ports.PIGEON_ID, Constants.Swerve.GYRO_INVERSION, Constants.Ports.CANIVORE_NAME),
                Constants.Swerve.SWERVE_KINEMATICS);

        locationLock = false;
        rotationLockPID = Constants.Swerve.autoRotate.getPIDController();

        camera = new PhotonCamera("23");
        camFilter = new PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                new Transform3d());
        maxDistanceCamToTarget = new TunableNumber("Max Cam->Target (m)", 6, Constants.TUNING_MODE);
    }

    public void enableRotationLock(double angle) {
        locationLock = true;

        rotationLockPID.setSetpoint(angle);
        rotationLockPID.calculate(getGyro().getAngle().getDegrees());
    }

    public Command rotateCommand(double angle) {
        return enableRotationLockCommand(angle).until(() -> {
            return Math.abs(getGyro().getAngle().getDegrees() - angle) < 2;
        }).andThen(disableRotationLockCommand());
    }

    public Command enableRotationLockCommand(double angle) {
        return runOnce(() -> enableRotationLock(angle));
    }

    public Command disableRotationLockCommand() {
        return runOnce(() -> disableRotationLock());
    }

    public void disableRotationLock() {
        locationLock = false;
    }

    @Override
    public void drive(Translation2d translation, double rotation, Alliance color) {
        if (locationLock) {
            rotation = rotationLockPID.calculate(getGyro().getAngle().getDegrees());
        }

        super.drive(translation, rotation, color);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!camera.isConnected())
            return;

        var pose = camFilter.update();

        if (!pose.isPresent() || pose.get().targetsUsed.size() < 2)
            return;

        var targetsCloseEnough = true;
        for (var target : pose.get().targetsUsed) {
            var transform = target.getBestCameraToTarget();
            double cameraToTagDistance = new Pose3d().transformBy(transform).getTranslation().getNorm();
            if (cameraToTagDistance > maxDistanceCamToTarget.get()) {
                targetsCloseEnough = false;
                break;
            }
        }
        if (targetsCloseEnough) {
            super.addVisionTargets(List.of(pose.get()));
        }

    }
}
