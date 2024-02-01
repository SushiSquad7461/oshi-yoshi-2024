package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
    private SendableChooser<Command> chooser;

    public AutoCommands(Swerve swerve) {
        NamedCommands.registerCommand("Nothing", new InstantCommand());

        AutoBuilder.configureHolonomic(
                swerve::getOdomPose,
                swerve::setOdomPose,
                swerve::getChassisSpeeds,
                swerve::drive,
                new HolonomicPathFollowerConfig(
                        Constants.Swerve.AUTO_TRANSLATION,
                        Constants.Swerve.AUTO_ROTATION,
                        Constants.Swerve.MODULE_TYPE.maxSpeed,
                        Constants.Swerve.DRIVE_BASE_RADIUS,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                swerve);

        chooser = new SendableChooser<>();

        chooser.addOption("nothing", new InstantCommand(() -> {
        }));

        chooser.addOption("Square", makeAuto("Square"));

        chooser.addOption("line", makeAuto("line"));

        SmartDashboard.putData("Auto Selecter", chooser);
    }

    private Command makeAuto(String path) {
        return new PathPlannerAuto(path);
    }

    public Command getAuto() {
        return chooser.getSelected();
    }
}
