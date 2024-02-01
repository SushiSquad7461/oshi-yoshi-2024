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
        public static final boolean TUNING_MODE = true;

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
                public static final double DRIVE_BASE_RADIUS = Math
                                .sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE);

                public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

                public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
                                20,
                                true, // Make true if we have a stroke
                                PIDConfig.getPid(0.8), // TODO: retune
                                MotorConfig.Mode.COAST);

                public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
                                60,
                                false,
                                PIDConfig.getPid(1.0, 0.68),
                                MotorConfig.Mode.BRAKE);

                public static final PIDConfig autoRotate = PIDConfig.getPid(0.1);

                public static final SDSModules MODULE_TYPE = SDSModules.MK4i;

                public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants
                                .generateConstants(
                                                new Rotation2d[] { // Tuned
                                                                Rotation2d.fromDegrees(0.351562),
                                                                Rotation2d.fromDegrees(15.996094),
                                                                Rotation2d.fromDegrees(124.189453),
                                                                Rotation2d.fromDegrees(62.578125)
                                                },
                                                MODULE_TYPE,
                                                true,
                                                DRIVE_CONFIG,
                                                ANGLE_CONFIG);
        }

        public static final class Manipulator {
                public static final MotorConfig KICKER_CONFIG = new MotorConfig(
                                25,
                                20,
                                true,
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
                                0,
                                20,
                                false,
                                PIDConfig.getPid(0.001),
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig SHOOTER_CONFIG_LEFT = new MotorConfig(
                                26,
                                20,
                                false,
                                PIDConfig.getPid(0.000080, 0.010000, 0.000180),
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig SHOOTER_CONFIG_RIGHT = new MotorConfig(
                                27,
                                20,
                                false,
                                PIDConfig.getPid(0.000080, 0.010000, 0.000180),
                                MotorConfig.Mode.BRAKE);

                public static final int ENCODER_ID = 0;// set this
                public static final double ENCODER_OFFSET = 0;// set this
                public static final double PIVOT_GEAR_RATIO = 0; // set ratio
                public static final double KS = 0;// set this
                public static final double KG = 0;// set this
                public static final double KV = 0;// set this
                public static final double KICKER_SPEED = 0.5;// set this
        }

        public static final class Intake {
                public static final double G = 0.0; // set later
                public static final int ENCODER_CHANNEL = 0; // set later
                public static final double ENCODER_ANGLE_OFFSET = 0.0; // set later
                public static final double INTAKE_GEAR_RATIO = 0.0; // set later
                public static final double SPIN_SPEED = 0.9; // set later
                public static final double ERROR_LIMIT = 0.0; // set later
                public static final double MAX_ERROR = 0.0; // set later

                public static final MotorConfig INTAKE_CONFIG = new MotorConfig(
                                21,
                                40, // set later
                                true, // spin motor
                                MotorConfig.Mode.COAST);

                public static final MotorConfig INDEXER_CONFIG = new MotorConfig(
                                24,
                                20, // set later
                                false, // spin motor
                                MotorConfig.Mode.COAST);

                public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
                                0, // set later
                                false, // position motor
                                PIDConfig.getPid(0.1),
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig UPRIGHT_ROLLERS_CONFIG = new MotorConfig(
                                22, // set canID later
                                20, // set later
                                false, // position motor
                                MotorConfig.Mode.COAST);
                                
        }
}
