package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Direction;

abstract public class Intake extends SubsystemBase {
    private final CANSparkMax indexerMotor;
    private final CANSparkMax intakeMotor;
    private final CANSparkMax uprightRollers;

    private final DigitalInput beamBreak;

    public Intake() {
        indexerMotor = Constants.Intake.INDEXER_CONFIG.createSparkMax();
        intakeMotor = Constants.Intake.INTAKE_CONFIG.createSparkMax();
        uprightRollers = Constants.Intake.UPRIGHT_ROLLERS_CONFIG.createSparkMax();

        beamBreak = new DigitalInput(1);
    }

    public boolean ringInIndexer() { return !beamBreak.get(); }

    public Command runIndexer() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Intake.INDEXER_SPEED);
            uprightRollers.set(Constants.Intake.UPRIGHT_ROLLERS_SPEED);
        });
    }

    public Command reverseIndexer() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Intake.INDEXER_SPEED * -1);
            uprightRollers.set(Constants.Intake.UPRIGHT_ROLLERS_SPEED * -1);
        });
    }

    public Command stopIndexer() {
        return runOnce(() -> {
            indexerMotor.set(0);
            uprightRollers.set(0);
        });
    }

    public Command runIntake() { 
        return runOnce(() -> intakeMotor.set(Constants.Intake.INTAKE_SPEED)); 
    }

    public Command reverseIntake() {
        return runOnce(() -> intakeMotor.set(-1 * Constants.Intake.INTAKE_SPEED));   
    }

    public Command stopIntake() {
        return runOnce(() -> intakeMotor.set(0.0));
    }

    public Command lowerIntake() { return Commands.none(); }
    public Command raiseIntake()  { return Commands.none(); }

    public Command changeState(IntakeState newState) {
        Command pivotCommand = newState.intakeExtended ? lowerIntake() : raiseIntake(); 

        Command intakeCommand = newState.intakeExtended ? (
            newState.direction == Direction.REVERSED ? reverseIntake() : runIntake()
        ) : stopIntake();

        Command indexerCommand;

        if (newState.direction == Direction.RUNNING) {
            indexerCommand = runIndexer();
        } else if (newState.direction == Direction.REVERSED) {
            indexerCommand = reverseIndexer();
        } else {
            indexerCommand = stopIndexer();
        }

        return pivotCommand.andThen(intakeCommand).andThen(indexerCommand);
    }

    @Override
    public void periodic() { 
        SmartDashboard.putBoolean("Ring in Indexer", ringInIndexer());

        SmartDashboard.putNumber("Intake current", intakeMotor.getOutputCurrent());
    }
}
