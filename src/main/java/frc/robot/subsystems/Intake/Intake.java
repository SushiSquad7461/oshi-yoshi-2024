package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

abstract public class Intake extends SubsystemBase {
    private final CANSparkMax indexerMotor;
    private final CANSparkMax intakeMotor;
    private final CANSparkMax uprightRollers;

    public Intake() {
        indexerMotor = Constants.Intake.INDEXER_CONFIG.createSparkMax();
        intakeMotor = Constants.Intake.INTAKE_CONFIG.createSparkMax();
        uprightRollers = Constants.Intake.UPRIGHT_ROLLERS_CONFIG.createSparkMax();
    }

    public Command runMotor() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Intake.SPIN_SPEED);
            intakeMotor.set(Constants.Intake.SPIN_SPEED);
            uprightRollers.set(Constants.Intake.SPIN_SPEED);
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            indexerMotor.set(0);
            intakeMotor.set(0);
            uprightRollers.set(0);
        });
    }

    public Command reverseMotor() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Intake.SPIN_SPEED * -1);
            intakeMotor.set(Constants.Intake.SPIN_SPEED * -1);
            uprightRollers.set(Constants.Intake.SPIN_SPEED * -1);
        });
    }
}
