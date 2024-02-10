package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class StateMachine extends Command {
    public enum RobotState {
        IDLE(IntakeState.IDLE, ShooterState.IDLE, ElevatorState.IDLE),
        INTAKE(IntakeState.INTAKE, ShooterState.IDLE, ElevatorState.IDLE),
        INDEX(IntakeState.INDEX, ShooterState.FEED, ElevatorState.IDLE),
        REVERSE(IntakeState.REVERSE, ShooterState.REVERSE, ElevatorState.IDLE),
        SHOOT(IntakeState.INDEX, ShooterState.SHOOT, ElevatorState.SPEAKER);

        public IntakeState intakeState;
        public ShooterState shooterState;
        private ElevatorState elevatorState;

        private RobotState(IntakeState intakeState, ShooterState shooterState, ElevatorState elevatorState) {
            this.intakeState = intakeState;
            this.shooterState = shooterState;
            this.elevatorState = elevatorState;
        }
    }

    private RobotState state;
    private Intake intake;
    private Shooter shooter;
    private Elevator elevator;

    public StateMachine(Intake intake, Shooter shooter, Elevator elevator) {
        this.intake = intake;
        this.shooter = shooter;
        this.elevator = elevator;
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

        if (intake.ringInIndexer() && state != RobotState.REVERSE) {
            scheduleNewState(RobotState.INDEX);
        } else if (!intake.ringInIndexer() && state == RobotState.INDEX) {
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
                elevator.changeState(newState.elevatorState),
                intake.changeState(newState.intakeState),
                shooter.changeState(newState.shooterState));
    }
}
