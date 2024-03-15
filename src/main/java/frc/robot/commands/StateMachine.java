package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class StateMachine extends Command {
    public enum RobotState {
        IDLE(IntakeState.IDLE, ShooterState.IDLE, IndexerState.IDLE, ElevatorState.IDLE),
        INTAKE(IntakeState.INTAKE, ShooterState.IDLE, IndexerState.INDEX, ElevatorState.IDLE),
        INDEX(IntakeState.IDLE, ShooterState.FEED, IndexerState.INDEX, ElevatorState.IDLE),
        REVERSE(IntakeState.REVERSE, ShooterState.REVERSE, IndexerState.REVERSE, ElevatorState.IDLE),
        CLIMB_UP(IntakeState.IDLE, ShooterState.IDLE, IndexerState.IDLE, ElevatorState.CLIMB),
        SUCK_IN(IntakeState.IDLE, ShooterState.REVERSE, IndexerState.IDLE, ElevatorState.IDLE),
        SHOOT_ANYWHERE(IntakeState.IDLE, ShooterState.SHOOT_ANYWHERE, IndexerState.IDLE, ElevatorState.IDLE),
        SHOOT_FENDOR(IntakeState.IDLE, ShooterState.SHOOT_FENDOR, IndexerState.IDLE, ElevatorState.SPEAKER),
        SHOOT_AMP(IntakeState.IDLE, ShooterState.SHOOT_AMP, IndexerState.IDLE, ElevatorState.AMP),
        SHOOT_TRAP(IntakeState.IDLE, ShooterState.SHOOT_TRAP, IndexerState.IDLE, ElevatorState.TRAP),
        SHOOT_STAGE(IntakeState.IDLE, ShooterState.SHOOT_STAGE, IndexerState.IDLE, ElevatorState.IDLE), // 40 10
        SPIT_OUT(IntakeState.IDLE, ShooterState.SPIT_OUT, IndexerState.IDLE, ElevatorState.IDLE),
        SHOOT_CENTER_LINE_AUTO(IntakeState.IDLE, ShooterState.SHOOT_CENTER_AUTO, IndexerState.IDLE, ElevatorState.IDLE);

        public IntakeState intakeState;
        public ShooterState shooterState;
        public IndexerState indexerState;
        public ElevatorState elevatorState;

        private RobotState(IntakeState intakeState, ShooterState shooterState, IndexerState indexerState, ElevatorState elevatorState) {
            this.indexerState = indexerState;
            this.intakeState = intakeState;
            this.shooterState = shooterState;
            this.elevatorState = elevatorState;
        }
    }

    private RobotState state;
    private Intake intake;
    private Shooter shooter;
    private Indexer indexer;
    private Elevator elevator;

    public StateMachine(Intake intake, Shooter shooter, Indexer indexer, Elevator elevator) {
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.elevator = elevator;
        state = RobotState.IDLE;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Robot State", state.toString());

        if (indexer.ringInIndexer() && state == RobotState.INTAKE) {
            scheduleNewState(RobotState.INDEX);
        }

        if (shooter.ringInShooter() && state == RobotState.INDEX) {
            shooter.reverseKicker().andThen(shooter.stopKicker()).andThen(changeState(RobotState.IDLE)).schedule();
        }

        if (state == RobotState.REVERSE && !indexer.ringInIndexer()) {
            scheduleNewState(RobotState.IDLE);
        }
    }

    public void scheduleNewState(RobotState newState) {
        changeState(newState).schedule();
    }

    public boolean inShooter() {
       return shooter.ringInShooter(); 
    }

    public Command changeState(RobotState newState) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                state = newState;
                System.out.println(newState.toString() + " scheduled");
            }),
            Commands.parallel(
                elevator.changeState(newState.elevatorState), 
                shooter.changeState(newState.shooterState)
            ),
            Commands.parallel(
                intake.changeState(newState.intakeState),
                indexer.changeState(newState.indexerState),
                shooter.changeKickerState(newState.shooterState)
            ),
            Commands.runOnce(() -> {
                System.out.println(newState.toString() + "  done");
            })
        );
    }
}
