package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;

import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.SmartDashboard.AllianceColor;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleTalon;
import SushiFrcLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.util.CameraSystem;


public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private boolean speakerLock;
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
            new SwerveModuleTalon[]{
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[0]),
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[1]),
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[2]),
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[3]),
            },
            new Pigeon(Constants.Ports.PIGEON_ID, Constants.Swerve.GYRO_INVERSION, Constants.Ports.CANIVORE_NAME),
            Constants.Swerve.SWERVE_KINEMATICS
        );

        locationLock = false;
        speakerLock = false;
        rotationLockPID = Constants.Swerve.autoRotate.getPIDController(); 


        cameraSystem = new CameraSystem(new String[] { "camera4", "camera2" },
                new Transform3d[] { new Transform3d(-0.2667, -0.24765, 0.2286, new Rotation3d(0, 0.16, 2.79252)),
                        new Transform3d(0.0889, 0.2794, 0.4699, new Rotation3d(0, 0.08726, 1.5707)) },
                "apriltags.json", field);
    }

    public void enableRotationLock(double angle) {
        locationLock = true;

        rotationLockPID.setSetpoint(angle);
        rotationLockPID.calculate(getGyro().getAngle().getDegrees());
    }

    public void disableRotationLock() {
        locationLock = false;
    }

    public void enableSpeakerLock() {
        speakerLock = true;
    }

    public void disableSpeakerLock() {
        speakerLock = false;
    }


    @Override
    public void drive(Translation2d translation, double rotation, Alliance color) {
        if (locationLock) {
            rotation = rotationLockPID.calculate(getGyro().getAngle().getDegrees());
        }

        if (speakerLock) {
            rotationLockPID.setSetpoint(odom.getEstimatedPosition().relativeTo(
                color == Alliance.Red ? Constants.FIELD_CONSTANTS.RED_SPEAKER : Constants.FIELD_CONSTANTS.BLUE_SPEAKER
            ).getRotation().getDegrees());

            rotation = rotationLockPID.calculate(rotation);
        }

        super.drive(translation, rotation, color);
    }

    @Override
    public void periodic(){
        ArrayList<EstimatedRobotPose> list = cameraSystem.getEstimatedPoses(getOdomPose());
        addVisionTargets(list);

        field.getObject("Estimated Poses").setPoses(
            list.stream().map(
                (estimate) -> estimate.estimatedPose.toPose2d()).collect(Collectors.toList()));
        super.periodic();
    }
}
