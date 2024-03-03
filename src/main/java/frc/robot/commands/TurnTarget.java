package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.BetaShooter;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;

public class TurnTarget extends Command{
    private final Swerve swerve;
    private final StateMachine stateMachine;
    private final boolean isRed;

    public TeleopShoot(StateMachine machine){
        swerve = Swerve.getInstance();
        this.stateMachine = machine;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        stateMachine.changeState(RobotState.IDLE);
        isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    @Override
    public void execute() {
        
    }

}
