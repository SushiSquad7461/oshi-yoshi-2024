package frc.robot;

import com.fasterxml.jackson.core.util.DefaultPrettyPrinter.Indenter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class AutoCommands {
    private SendableChooser<Command> chooser;

    public AutoCommands(Swerve swerve, StateMachine stateMachine) {
        NamedCommands.registerCommand("Nothing", new InstantCommand());

        NamedCommands.registerCommand("suck_in", stateMachine.shooter.reverseKicker().andThen(new WaitCommand(0.2)).andThen(
            stateMachine.shooter.stopKicker()
        ));

        NamedCommands.registerCommand("shoot_fendor", 
            stateMachine.changeState(RobotState.SHOOT_FENDOR).andThen(
                new WaitCommand(0.08)
            ).andThen(stateMachine.changeState(RobotState.IDLE))
        );

        // NamedCommands.registerCommand("schedule_intake", new ParallelCommandGroup(
        //     intake.changeState(IntakeState.INTAKE),
        //     indexer.changeState(IndexerState.INDEX),
        //     shooter.changeKickerState(ShooterState.FEED)
        // ));

        // NamedCommands.registerCommand("spit_out",  new ParallelCommandGroup(
        //     intake.changeState(IntakeState.IDLE),
        //     indexer.changeState(IndexerState.IDLE),
        //     shooter.changeKickerState(ShooterState.SHOOT_CENTER_AUTO)
        // ));

        // NamedCommands.registerCommand("shoot_center_amp",  new ParallelCommandGroup(
        //     intake.changeState(IntakeState.IDLE),
        //     indexer.changeState(IndexerState.IDLE),
        //     shooter.changeKickerState(ShooterState.SHOOT_CENTER_AUTO)
        // ));

        NamedCommands.registerCommand("schedule_intake", stateMachine.changeState(RobotState.INTAKE_AUTOS
            ).andThen(Commands.waitUntil(() -> stateMachine.inShooter())).andThen(stateMachine.shooter.reverseKicker()).andThen(stateMachine.shooter.stopKicker()).andThen(stateMachine.changeState(RobotState.IDLE))
        );

        NamedCommands.registerCommand("spit_out",  stateMachine.changeState(RobotState.SPIT_OUT));

        NamedCommands.registerCommand("shoot_center_amp", 
        stateMachine.changeState(RobotState.SHOOT_CENTER_LINE_AUTO).andThen(
            new WaitCommand(0.05)
        ));

        AutoBuilder.configureHolonomic(
                swerve::getOdomPose,
                swerve::setOdomPose,
                swerve::getChassisSpeeds,
                swerve::driveChassis,
                new HolonomicPathFollowerConfig(
                        Constants.Swerve.AUTO_TRANSLATION,
                        Constants.Swerve.AUTO_ROTATION,
                        Constants.Swerve.MODULE_TYPE.maxSpeed,
                        Constants.Swerve.DRIVE_BASE_RADIUS,
                        new ReplanningConfig(true, true)),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                swerve
        );

        chooser = new SendableChooser<>();

        chooser.addOption("nothing", new InstantCommand(() -> {
        }));

        chooser.addOption("Square", makeAuto("Square"));

        chooser.addOption("Line", makeAuto("Line"));

        chooser.addOption("TwoPieceCenterLine", makeAuto("TwoPieceCenter"));

        chooser.addOption("3 Piece Center", makeAuto("3 piece"));

        chooser.addOption("Center Amp", makeAuto("CenterAmp"));

        chooser.addOption("Center Source", makeAuto("CenterSource"));

        chooser.addOption("2 piece center amp side", makeAuto("2 piece center amp side"));

        SmartDashboard.putData("Auto Selecter", chooser);
    }

    private Command makeAuto(String path) {
        return new PathPlannerAuto(path);
    }

    public Command getAuto() {
        return chooser.getSelected();
    }
}
