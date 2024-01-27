// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Swerve.SwerveConstants.SDSModules;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final boolean TUNING_MODE = false;

    public static final class OI {
        public static final double STICK_DEADBAND = 0.1;
    }

    public static class Ports {
        public static final String CANIVORE_NAME = "Sussy Squad";
        public static final int PIGEON_ID = 13;
    }

    public static final class Swerve {
        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        public static final PIDConstants AUTO_TRANSLATION = new PIDConstants(0); // TODO: find pid
        public static final PIDConstants AUTO_ROTATION = new PIDConstants(0); // TODO: find pid

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(28);
        public static final double WHEEL_BASE = Units.inchesToMeters(28);
        public static final double DRIVE_BASE_RADIUS = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0));

        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
                20,
                false, // Make true if we have a stroke
                PIDConfig.getPid(0.5), // TODO: retune
                MotorConfig.Mode.COAST);

        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
                60,
                false,
                PIDConfig.getPid(0.0, 0.0), // TODO: retune
                MotorConfig.Mode.BRAKE);

        public static final PIDConfig autoRotate = PIDConfig.getPid(0.1);

        public static final SDSModules MODULE_TYPE = SDSModules.MK4i;

        public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants.generateConstants(
                new Rotation2d[] { // Tuned
                        Rotation2d.fromDegrees(8.085938),
                        Rotation2d.fromDegrees(20.654297),
                        Rotation2d.fromDegrees(298.212891),
                        Rotation2d.fromDegrees(232.382812)
                },
                MODULE_TYPE,
                true,
                DRIVE_CONFIG,
                ANGLE_CONFIG);
    }
}
