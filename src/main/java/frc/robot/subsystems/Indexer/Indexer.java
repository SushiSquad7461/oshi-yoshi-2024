package frc.robot.subsystems.Indexer;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Direction;

public class Indexer {
    private final CANSparkMax indexerMotor;
    private final CANSparkMax uprightRollers;

    public Indexer() {
        indexerMotor = Constants.Indexer.INDEXER_CONFIG.createSparkMax();
        uprightRollers = Constants.Indexer.UPRIGHT_ROLLERS_CONFIG.createSparkMax();
    }

    public Command runIndexer() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Intake.INDEXER_SPEED);
            uprightRollers.set(Constants.Intake.UPRIGHT_ROLLERS_SPEED);
        });
    }
}
