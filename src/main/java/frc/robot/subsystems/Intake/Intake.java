package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Direction;

abstract public class Intake extends SubsystemBase {
    protected final CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = Constants.Intake.INTAKE_CONFIG.createSparkMax();
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

    public Command lowerIntake() {
        return Commands.none();
    }

    public Command raiseIntake() {
        return Commands.none();
    }

    public Command changeState(IntakeState newState) {
        Command pivotCommand = newState.intakeExtended ? lowerIntake() : raiseIntake();

        Command intakeCommand = newState.intakeExtended
                ? (newState.direction == Direction.REVERSED ? reverseIntake() : runIntake())
                : stopIntake();
        return pivotCommand.andThen(intakeCommand);
    }

    @Override
    public void periodic() {
        intakeMotor.set(.8);
        SmartDashboard.putNumber("Intake current", intakeMotor.getOutputCurrent());
    }
}
