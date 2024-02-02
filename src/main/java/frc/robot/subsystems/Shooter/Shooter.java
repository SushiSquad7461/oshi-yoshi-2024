package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.robot.util.Direction;

abstract public class Shooter extends SubsystemBase {
    private final CANSparkMax kicker;
    private final CANSparkMax shooterLeft;
    private final CANSparkMax shooterRight;

    private final PIDTuning tuning;
    private final TunableNumber shooterSpeed;

    public Shooter() {
        shooterSpeed = new TunableNumber("Shooter Speed", 0, Constants.TUNING_MODE);
        kicker = Manipulator.KICKER_CONFIG.createSparkMax();
        shooterLeft = Manipulator.SHOOTER_CONFIG_LEFT.createSparkMax();
        shooterRight = Manipulator.SHOOTER_CONFIG_RIGHT.createSparkMax();
        shooterLeft.follow(shooterRight, false);

        tuning = new PIDTuning("Shooter", Manipulator.SHOOTER_CONFIG_RIGHT.pid, Constants.TUNING_MODE);
    }

    public Command runKicker() {
        return runOnce(() -> {
            kicker.set(Manipulator.KICKER_SPEED);
        });
    }

    public Command reverseKicker() {
        return runOnce(() -> {
            kicker.set(-Manipulator.KICKER_SPEED);
        });
    }

    public Command stopKicker() {
        return runOnce(() -> {
            kicker.set(0);
        });
    }

    public Command runShooter(double speed) {
        return run(() -> {
            shooterRight.set(0.8);
            // shooterSpeed.setDefault(speed);
            // shooterRight.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
        });
    }

    public boolean shooterAtSpeed() {
        return (Math.abs(shooterRight.getEncoder().getVelocity() - shooterSpeed.get()) < 5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterRight.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter current", shooterRight.getOutputCurrent());

        tuning.updatePID(shooterRight);

        if (Constants.TUNING_MODE) {
            // shooterRight.set(0.5);
            // shooterRight.getPIDController().setReference(shooterSpeed.get(),
            // CANSparkBase.ControlType.kVelocity);
        }
    }

    public Command changeState(ShooterState newState) {
        Command kickerCommmand;

        if (newState.kickerDirection == Direction.RUNNING) {
            kickerCommmand = runKicker();
        } else if (newState.kickerDirection == Direction.REVERSED) {
            kickerCommmand = reverseKicker();
        } else {
            kickerCommmand = stopKicker();
        }

        return runShooter(newState.runShooter ? 4000 : 0).andThen(kickerCommmand);
    }
}
