// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Controllers.OI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.AlphaIntake;
import frc.robot.subsystems.Intake.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  OI oi;
  Swerve swerve;
  Intake intake;

  public RobotContainer() {
    oi = OI.getInstance();
    // swerve = Swerve.getInstance();
    intake = AlphaIntake.getInstance();
    configureBindings();
  }

  private void configureBindings() {
    oi.getDriverController().leftBumper().onTrue(intake.runMotor()).onFalse(intake.stopMotor());
    oi.getDriverController().rightBumper().onTrue(intake.runMotor()).onFalse(intake.stopMotor());
    // swerve.setDefaultCommand(new TeleopSwerveDrive(
    // swerve,
    // () -> oi.getDriveTrainTranslationX(),
    // () -> oi.getDriveTrainTranslationY(),
    // () -> oi.getDriveTrainRotation()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
