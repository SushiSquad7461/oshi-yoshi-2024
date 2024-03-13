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

                public static final double MAX_ERROR = 1.0;
        }

        public static final class Swerve {
                public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

                public static final PIDConstants AUTO_TRANSLATION = new PIDConstants(13); // TODO: find pid
                public static final PIDConstants AUTO_ROTATION = new PIDConstants(5);

                /* Drivetrain Constants */
                public static final double TRACK_WIDTH = Units.inchesToMeters(23);
                public static final double WHEEL_BASE = Units.inchesToMeters(23);

                public static final double DRIVE_BASE_RADIUS = Math
                                .sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2;

                public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0));

                public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
                                20,
                                false, // Make true if we have a stroke
                                PIDConfig.getPid(0.1), // TODO: retune
                                MotorConfig.Mode.COAST);

                public static final MotorConfig ANGLE_FLIPPED_CONFIG = new MotorConfig(
                                20,
                                true, // Make true if we have a stroke
                                PIDConfig.getPid(0.1), // TODO: retune
                                MotorConfig.Mode.COAST);

                public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
                                40,
                                true,
                                PIDConfig.getPid(0.2, 0.68),
                                MotorConfig.Mode.BRAKE);

                public static final MotorConfig DRIVE_FLIPPED_CONFIG = new MotorConfig(
                                40,
                                false,
                                PIDConfig.getPid(0.2, 0.68),
                                MotorConfig.Mode.BRAKE);

                public static final PIDConfig autoRotate = PIDConfig.getPid(0.02);

                public static final SDSModules MODULE_TYPE = SDSModules.MK4i;

                public static final boolean SWERVE_TUNING_MODE = false;

                public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants[] {
                                new SwerveModuleConstants(0, Rotation2d.fromDegrees(346.81), MODULE_TYPE,
                                                SWERVE_TUNING_MODE, DRIVE_CONFIG, ANGLE_CONFIG),
                                new SwerveModuleConstants(1, Rotation2d.fromDegrees(21.09), MODULE_TYPE,
                                                SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
                                new SwerveModuleConstants(2, Rotation2d.fromDegrees(25.48), MODULE_TYPE,
                                                SWERVE_TUNING_MODE, DRIVE_CONFIG, ANGLE_CONFIG),
                                new SwerveModuleConstants(3, Rotation2d.fromDegrees(126.29), MODULE_TYPE,
                                                SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
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
                                PIDConfig.getPid(0.02),
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
                                PIDConfig.getPid(0.00009, 0.0, 0.000174),
                                MotorConfig.Mode.BRAKE);

                public static final int ENCODER_ID = 4;// set this
                public static final int BEAM_BREAK_ID = 3;
                public static final double ENCODER_OFFSET = -19.0;// set this
                public static final double PIVOT_GEAR_RATIO = 66.666; // set ratio
                public static final double KS = 0;// set this
                public static final double KG = 0;// set this
                public static final double KV = 0;// set this
                public static final double KICKER_SPEED = 0.6;// set this

                public static final double SHOOTER_SPEED = 5000;

                public static final double PIVOT_AMP_ANGLE = 20; // find angle
                public static final double PIVOT_TRAP_ANGLE = 0; // find angle
                public static final double PIVOT_STAGE_ANGLE = -41; // 31.5
                public static final double PIVOT_IDLE = -60;
                public static final double SHOOTER_ERROR = 300;
                public static final double PIVOT_ERROR = 1.0;
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
                public static final double INDEXER_SPEED = 0.7;
                public static final MotorConfig INDEXER_CONFIG = new MotorConfig(
                                24,
                                40, // set later
                                false, // spin motor
                                MotorConfig.Mode.COAST);
                public static final MotorConfig UPRIGHT_ROLLERS_CONFIG = new MotorConfig(
                                23, // set canID later
                                20,
                                true, // position motor
                                MotorConfig.Mode.COAST);
        }

        public static final class Intake {
                public static final double G = 0.25; // retune
                public static final int ENCODER_CHANNEL = 1;
                public static final double ENCODER_ANGLE_OFFSET = -58; //60
                public static final double INTAKE_GEAR_RATIO = 21.701;

                public static final double INTAKE_SPEED = 0.9;

                public static final double ERROR_LIMIT = 1.0;
                public static final double MAX_ERROR = 3.0;

                public static final double RAISED_POS = 110;
                public static final double LOWERED_POS = -9; //-26

                public static final MotorConfig INTAKE_CONFIG = new MotorConfig(
                                21,
                                40, // set later
                                true, // spin motor
                                MotorConfig.Mode.COAST);

                public static final MotorConfig PIVOT_CONFIG = new MotorConfig(
                                22,
                                20,
                                true,
                                PIDConfig.getPid(0.009, 0.0, 0.0), // p:0.012d:0.6
                                MotorConfig.Mode.BRAKE);
        }
}
