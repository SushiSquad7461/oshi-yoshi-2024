// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Swerve.SwerveKinematics;
import SushiFrcLib.Swerve.SwerveModules.SDSModules;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModules2024;

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
        public static final double WHEEL_DIAMATER = Units.inchesToMeters(4);

        public static final SwerveKinematics SWERVE_KINEMATICS = new SwerveKinematics(
          new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
        );


        public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
            20,
            false, // Make true if we have a stroke
            PIDConfig.getPid(0.1), 
            MotorConfig.Mode.BRAKE 
        );

        public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
            60,
            false, 
            PIDConfig.getPid(0.1, 0.0458), 
            MotorConfig.Mode.BRAKE 
        );

        public static final PIDConfig autoRotate = PIDConfig.getPid(0.1);

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 5; // 5 meters per second
        public static final double MAX_ACCELERATION = 4; // 4 meters per second squared
        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI; // radians per second
        public static final double MAX_ANGULAR_ACCELERATION = 20; // TODO: tune

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        public static final boolean SWERVE_TUNNING_MODE = false;

        public static final SwerveModuleConstants MOD0_CONSTANTS = new SwerveModules2024(
            0, 
            159.9, 
            SDSModules.MK4i
        );

        public static final SwerveModuleConstants MOD1_CONSTANTS = new SwerveModules2024(
            1, 
            338.291, 
            SDSModules.MK4i
        );

        public static final SwerveModuleConstants MOD2_CONSTANTS = new SwerveModules2024(
            2, 
            178.33, 
            SDSModules.MK4i
        );

        public static final SwerveModuleConstants MOD3_CONSTANTS = new SwerveModules2024(
            3, 
            104.58, 
            SDSModules.MK4i
        );
    }
}
