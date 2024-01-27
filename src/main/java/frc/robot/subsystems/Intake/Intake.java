package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

abstract public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotorLeft;
    private final CANSparkMax intakeMotorRight;
    private final CANSparkMax uprightRollers;

    public Intake() {
        intakeMotorLeft = Constants.Intake.INTAKE_LEFT_CONFIG.createSparkMax();
        intakeMotorRight = Constants.Intake.INTAKE_RIGHT_CONFIG.createSparkMax();
        uprightRollers = Constants.Intake.UPRIGHT_ROLLERS_CONFIG.createSparkMax();
    }

    public Command runMotor() {
        return runOnce(() -> {
            intakeMotorLeft.set(Constants.Intake.SPIN_SPEED);
            intakeMotorRight.set(Constants.Intake.SPIN_SPEED);
            uprightRollers.set(Constants.Intake.SPIN_SPEED);
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            intakeMotorLeft.set(0);
            intakeMotorRight.set(0);
            uprightRollers.set(0);
        });
    }

    public Command reverseMotor() {
        return runOnce(() -> {
            intakeMotorLeft.set(Constants.Intake.SPIN_SPEED * -1);
            intakeMotorRight.set(Constants.Intake.SPIN_SPEED * -1);
            uprightRollers.set(Constants.Intake.SPIN_SPEED * -1);
        });
    }
}
