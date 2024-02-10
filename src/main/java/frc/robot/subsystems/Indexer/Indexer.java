package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Direction;

public class Indexer extends SubsystemBase {
    private final CANSparkMax indexerMotor;
    private final CANSparkMax uprightRollers;
    private static Indexer instance;
    public static DigitalInput beamBreak;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }

        return instance;
    }

    public Indexer() {
        beamBreak = new DigitalInput(Constants.Indexer.BEAM_BREAK_ID);
        indexerMotor = Constants.Indexer.INDEXER_CONFIG.createSparkMax();
        uprightRollers = Constants.Indexer.UPRIGHT_ROLLERS_CONFIG.createSparkMax();
        beamBreak = new DigitalInput(Constants.Indexer.BEAM_BREAK);
    }

    public boolean ringInIndexer() {
        return !beamBreak.get();
    }

    public Command runIndexer() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Indexer.INDEXER_SPEED);
            uprightRollers.set(Constants.Indexer.UPRIGHT_ROLLERS_SPEED);
        });
    }

    public Command reverseIndexer() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Indexer.INDEXER_SPEED * -1);
            uprightRollers.set(Constants.Indexer.UPRIGHT_ROLLERS_SPEED * -1);
        });
    }

    public Command stopIndexer() {
        return runOnce(() -> {
            indexerMotor.set(0);
            uprightRollers.set(0);
        });
    }

    public Command changeState(IndexerState newState) {
        Command indexerCommand;

        if (newState.direction == Direction.RUNNING) {
            indexerCommand = runIndexer();
        } else if (newState.direction == Direction.REVERSED) {
            indexerCommand = reverseIndexer();
        } else {
            indexerCommand = stopIndexer();
        }

        return indexerCommand;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Ring in Indexer", ringInIndexer());
        SmartDashboard.putNumber("Indexer current", indexerMotor.getOutputCurrent());
    }
}
