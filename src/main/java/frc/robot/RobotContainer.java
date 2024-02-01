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
import frc.robot.subsystems.Shooter.AlphaShooter;
import frc.robot.subsystems.Shooter.Shooter;

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
  Shooter shooter;

  public RobotContainer() {
    oi = OI.getInstance();
    swerve = Swerve.getInstance();
    shooter = AlphaShooter.getInstance();
    intake = AlphaIntake.getInstance();
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(new TeleopSwerveDrive(
    swerve,
    () -> oi.getDriveTrainTranslationX(),
    () -> oi.getDriveTrainTranslationY(),
    () -> oi.getDriveTrainRotation()
    ));

    oi.getDriverController().a().onTrue(shooter.runKicker()).onFalse(shooter.stopKicker());
    oi.getDriverController().b().onTrue(shooter.runShooter(4000)).onFalse(shooter.runShooter(0));

    oi.getDriverController().x().onTrue(AlphaIntake.getInstance().runMotor())
        .onFalse(AlphaIntake.getInstance().stopMotor());

    oi.getDriverController().y().onTrue(AlphaIntake.getInstance().reverseMotor())
        .onFalse(AlphaIntake.getInstance().stopMotor());
  }
 
  public Command getAutonomousCommand() {
    return null;
  }
}
