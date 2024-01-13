// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Swerve.CustomSwerve.SwerveKinematics;
import SushiFrcLib.Swerve.SwerveConstants.SDSModules;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(28);
        public static final double WHEEL_BASE = Units.inchesToMeters(28);

        public static final SwerveKinematics SWERVE_KINEMATICS = new SwerveKinematics(
          new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
        );

        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
            20,
            false, // Make true if we have a stroke
            PIDConfig.getPid(0.01), 
            MotorConfig.Mode.COAST 
        );

        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            60,
            false, 
            PIDConfig.getPid(0.1, 0.0458), 
            MotorConfig.Mode.BRAKE 
        );

        public static final PIDConfig autoRotate = PIDConfig.getPid(0.1);

         public static final boolean SWERVE_TUNNING_MODE = true;

        public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants.generateConstants(
            // new Rotation2d[]{
            //     Rotation2d.fromDegrees(159.9), 
            //     Rotation2d.fromDegrees(338.291),
            //     Rotation2d.fromDegrees(178.33),
            //     Rotation2d.fromDegrees(104.58)
            // },
            new Rotation2d[]{
                Rotation2d.fromDegrees(158.8), 
                Rotation2d.fromDegrees(331.52),
                Rotation2d.fromDegrees(176.0),
                Rotation2d.fromDegrees(104.76)
            },
            SDSModules.MK4i,
            SWERVE_TUNNING_MODE,
            DRIVE_CONFIG,
            ANGLE_CONFIG
        );
    }
}
