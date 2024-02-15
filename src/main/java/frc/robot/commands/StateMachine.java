package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class StateMachine extends Command {
    public enum RobotState {
        IDLE(IntakeState.IDLE, ShooterState.IDLE, IndexerState.IDLE),
        INTAKE(IntakeState.INTAKE, ShooterState.IDLE, IndexerState.INDEX),
        INDEX(IntakeState.IDLE, ShooterState.FEED, IndexerState.INDEX),
        REVERSE(IntakeState.REVERSE, ShooterState.REVERSE, IndexerState.REVERSE),
        SHOOT_ANYWHERE(IntakeState.IDLE, ShooterState.SHOOT_ANYWHERE, IndexerState.IDLE), // should i set this to idle?
        SHOOT_FENDOR(IntakeState.IDLE, ShooterState.SHOOT_FENDOR, IndexerState.IDLE),
        SHOOT_AMP(IntakeState.IDLE, ShooterState.SHOOT_AMP, IndexerState.IDLE),
        SHOOT_TRAP(IntakeState.IDLE, ShooterState.SHOOT_TRAP, IndexerState.IDLE),
        SHOOT_STAGE(IntakeState.IDLE, ShooterState.SHOOT_STAGE, IndexerState.IDLE);

        public IntakeState intakeState;
        public ShooterState shooterState;
        public IndexerState indexerState;

        private RobotState(IntakeState intakeState, ShooterState shooterState, IndexerState indexerState) {
            this.indexerState = indexerState;
            this.intakeState = intakeState;
            this.shooterState = shooterState;
        }
    }

    private RobotState state;
    private Intake intake;
    private Shooter shooter;
    private Indexer indexer;

    public StateMachine(Intake intake, Shooter shooter, Indexer indexer) {
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        scheduleNewState(RobotState.IDLE);
    }

    public void end(boolean interrupted) {
        System.out.println("State Machine Command End");
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Robot State", state.toString());

<<<<<<< HEAD
        if (indexer.ringInIndexer() && state == RobotState.INTAKE) {
            scheduleNewState(RobotState.INDEX);
        }

        if (!indexer.ringInIndexer() && state == RobotState.INDEX) {
            scheduleNewState(RobotState.IDLE);
        }

        if (state == RobotState.REVERSE && !indexer.ringInIndexer()) {
=======
        if (indexer.ringInIndexer() && state != RobotState.REVERSE) {
            scheduleNewState(RobotState.INDEX);
        } else if (!indexer.ringInIndexer() && state == RobotState.INDEX) {
>>>>>>> 5b79430 (fixy beta code)
            scheduleNewState(RobotState.IDLE);
        }
    }

    private void scheduleNewState(RobotState newState) {
        changeState(newState).schedule();
    }

    public Command changeState(RobotState newState) {
        return Commands.parallel(
                Commands.runOnce(() -> {
                    state = newState;
                    System.out.println(newState.toString() + " scheduled");
                }),
                intake.changeState(newState.intakeState),
                indexer.changeState(newState.indexerState),
                shooter.changeState(newState.shooterState));
    }
}
