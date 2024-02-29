package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter.BetaShooter;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;

public class TeleopShoot extends Command{
    private final Swerve swerve;
    private final StateMachine stateMachine;
    private final Shooter shooter;
    private boolean isFinished;
    private boolean hasRotated;

    public TeleopShoot(StateMachine machine){
        swerve = Swerve.getInstance();
        shooter = BetaShooter.getInstance();
        this.stateMachine = machine;
        this.isFinished = false;
        this.hasRotated = false;
    }

    @Override
    public void initialize() {
        stateMachine.changeState(RobotState.IDLE);
    }

    @Override
    public void execute() {
        //rotate
        
        if (this.hasRotated) {
            shooter.setManipulatorPos(0);
            shooter.setShooterSpeed(0);
        }
    }

}
