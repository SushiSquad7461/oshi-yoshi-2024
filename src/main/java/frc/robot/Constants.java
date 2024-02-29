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

        public enum Robot {
                ALPHA,
                BETA;
        }

        public static final Robot ROBOT = Robot.BETA;

        public static final class OI {
                public static final double STICK_DEADBAND = 0.1;
        }

        public static class Ports {
                public static final String CANIVORE_NAME = "Sussy Squad";
                public static final int PIGEON_ID = 13;
        }

        public static class Elevator {
                public static final MotorConfig ELEVATOR_LEFT = new MotorConfig(
                                29,
                                10,
                                true,
                                PIDConfig.getPid(0.12),
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig ELEVATOR_RIGHT = new MotorConfig(
                                30,
                                10,
                                true,
                                PIDConfig.getPid(0.12, 0.0, 0.0),
                                MotorConfig.Mode.BRAKE);

                public static final double G_DOWN = 0;
                public static final double G_UP = 0;
                public static final double MAX_ERROR = 0;
        }

        public static final class Swerve {
                public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

                public static final PIDConstants AUTO_TRANSLATION = new PIDConstants(0.2); // TODO: find pid
                public static final PIDConstants AUTO_ROTATION = new PIDConstants(0.5); // TODO: find pid

                /* Drivetrain Constants */
                public static final double TRACK_WIDTH = Units.inchesToMeters(28);
                public static final double WHEEL_BASE = Units.inchesToMeters(28);
                public static final double DRIVE_BASE_RADIUS = Math
                                .sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE);

                public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0));

                public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
                        20,
                        false, // Make true if we have a stroke
                        PIDConfig.getPid(0.07), // TODO: retune
                        MotorConfig.Mode.COAST
                );


                public static final MotorConfig ANGLE_FLIPPED_CONFIG = new MotorConfig(
                        20,
                        true, // Make true if we have a stroke
                        PIDConfig.getPid(0.07), // TODO: retune
                        MotorConfig.Mode.COAST
                );

                public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
                        60,
                false,
                        PIDConfig.getPid(0.2, 0.68),
                        MotorConfig.Mode.BRAKE
                );

                public static final MotorConfig DRIVE_FLIPPED_CONFIG = new MotorConfig(
                        60,
                        true,
                        PIDConfig.getPid(0.2, 0.68),
                        MotorConfig.Mode.BRAKE
                );

                public static final PIDConfig autoRotate = PIDConfig.getPid(0.1);

                public static final SDSModules MODULE_TYPE = SDSModules.MK4i;

                // public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = SwerveModuleConstants
                //                 .generateConstants(
                //                                 new Rotation2d[] { // Tuned
                //                                                 Rotation2d.fromDegrees(0.0), // 356.22
                //                                                 Rotation2d.fromDegrees(0.0), // 347.16
                //                                                 Rotation2d.fromDegrees(0.0), // 55.68
                //                                                 Rotation2d.fromDegrees(0.0) //131.0
                //                                 },
                //         MODULE_TYPE,
                //         true,
                //         DRIVE_CONFIG,
                //         ANGLE_CONFIG
                // );

                public static final boolean SWERVE_TUNING_MODE = false;

                public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants[] {
                        new SwerveModuleConstants(0, Rotation2d.fromDegrees(166.8), MODULE_TYPE, SWERVE_TUNING_MODE, DRIVE_CONFIG, ANGLE_CONFIG),
                        new SwerveModuleConstants(1, Rotation2d.fromDegrees(201.7), MODULE_TYPE, SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
                        new SwerveModuleConstants(2, Rotation2d.fromDegrees(205.05), MODULE_TYPE, SWERVE_TUNING_MODE, DRIVE_CONFIG, ANGLE_CONFIG),
                        new SwerveModuleConstants(3, Rotation2d.fromDegrees(312.2), MODULE_TYPE, SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
                };
        }

        public static final class Manipulator {
                public static final MotorConfig KICKER_CONFIG = new MotorConfig(
                                25,
                                40,
                                false,
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
                                26,
                                20,
                                false,
                                PIDConfig.getPid(0.05),
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig SHOOTER_CONFIG_TOP = new MotorConfig(
                                27,
                                40,
                                true,
                                PIDConfig.getPid(0.0),
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig SHOOTER_CONFIG_BOTTOM = new MotorConfig(
                                28,
                                40,
                                true,
                                PIDConfig.getPid(0.0001, 0.0, 0.000185),
                                MotorConfig.Mode.BRAKE);

                public static final int ENCODER_ID = 4;// set this
                public static final int BEAM_BREAK_ID = 3;
                public static final double ENCODER_OFFSET = 0.0;// set this
                public static final double PIVOT_GEAR_RATIO = 75.0; // set ratio
                public static final double KS = 0;// set this
                public static final double KG = 0;// set this
                public static final double KV = 0;// set this
                public static final double KICKER_SPEED = 1.0;// set this

                public static final double SHOOTER_SPEED = 5000;

                public static final double PIVOT_FENDOR_ANGLE = 0; // find angle
                public static final double PIVOT_AMP_ANGLE = 0; // find angle
                public static final double PIVOT_TRAP_ANGLE = 0; // find angle
                public static final double PIVOT_STAGE_ANGLE = 0; // find angle
                public static final double PIVOT_IDLE = -60;// resting
                public static final double SHOOTER_ERROR = 50;
                public static final double PIVOT_ERROR = .1;
        }

        public static final class Indexer {
                public static int BEAM_BREAK;

                static {
                        switch (ROBOT) {
                                case ALPHA:
                                        BEAM_BREAK = 1;
                                        break;
                                default:
                                        BEAM_BREAK = 0;

                        }
                }

                public static final double UPRIGHT_ROLLERS_SPEED = 0.9;
                public static final double INDEXER_SPEED = 0.9;
                public static final MotorConfig INDEXER_CONFIG = new MotorConfig(
                                24,
                                40, // set later
                                false, // spin motor
                                MotorConfig.Mode.COAST);
                public static final MotorConfig UPRIGHT_ROLLERS_CONFIG = new MotorConfig(
                                23, // set canID later
                                30, // set later
                                false, // position motor
                                MotorConfig.Mode.COAST);
        }

        public static final class Intake {
                public static final double G = 0.3; // retune
                public static final int ENCODER_CHANNEL = 1;
                public static final double ENCODER_ANGLE_OFFSET = -55.1;
                public static final double INTAKE_GEAR_RATIO = 21.701;

                public static final double INTAKE_SPEED = 0.9;

                public static final double ERROR_LIMIT = 3.0;
                public static final double MAX_ERROR = 3.0;

                public static final double RAISED_POS = 105;
                public static final double LOWERED_POS = -12.67;

                public static final MotorConfig INTAKE_CONFIG = new MotorConfig(
                                21,
                                40, // set later
                                true, // spin motor
                                MotorConfig.Mode.COAST);

                public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
                                22,
                                20, // set later
                                true, // position motor
                                PIDConfig.getPid(0.005),
                                MotorConfig.Mode.BRAKE);
        }
}
