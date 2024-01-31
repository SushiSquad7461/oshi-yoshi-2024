// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Controllers.OI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LED;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  OI oi;
  Swerve swerve;
  LED led;

  public RobotContainer() {
    oi = OI.getInstance();
    led = LED.getInstance();
    led.setRace2(1, new LED.Color[]{LED.Color.DarkPink, LED.Color.Black});
    // swerve = Swerve.getInstance();
    // configureBindings();
  }

  private void configureBindings() {
    // swerve.setDefaultCommand(new TeleopSwerveDrive(
    //   swerve,
    //   () -> oi.getDriveTrainTranslationX(),
    //   () -> oi.getDriveTrainTranslationY(),
    //   () -> oi.getDriveTrainRotation()
    // ));
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
