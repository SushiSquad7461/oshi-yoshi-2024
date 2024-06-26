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
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;

    private final PIDTuning tuning;
    protected final TunableNumber shooterSpeed;

    public Shooter() {
        kicker = Manipulator.KICKER_CONFIG.createSparkMax();
        shooterTop = Manipulator.SHOOTER_CONFIG_TOP.createSparkMax();
        shooterBottom = Manipulator.SHOOTER_CONFIG_BOTTOM.createSparkMax();
        shooterTop.follow(shooterBottom, false);

        tuning = new PIDTuning("Shooter", Manipulator.SHOOTER_CONFIG_BOTTOM.pid, Constants.TUNING_MODE);
        shooterSpeed = new TunableNumber("Shooter Speed", 0, Constants.TUNING_MODE);
    }

    abstract public boolean ringInShooter();

    public Command runKicker() {
        return runOnce(() -> kicker.set(Manipulator.KICKER_SPEED));
    }

    public Command reverseKicker() {
        return runOnce(() -> kicker.set(-0.2));
    }

    public Command reverseFastKicker() {
        return runOnce(() -> kicker.set(-0.6));
    }

    public Command stopKicker() {
        return runOnce(() -> kicker.set(0));
    }

    public boolean shooterAtSpeed(double speed) {
        return (Math.abs(shooterBottom.getEncoder().getVelocity() - speed) < Manipulator.SHOOTER_ERROR);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterBottom.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Top", shooterTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("Error", shooterTop.getOutputCurrent());

        tuning.updatePID(shooterBottom);
        shooterBottom.getPIDController().setReference(shooterSpeed.get(), CANSparkBase.ControlType.kVelocity);
    }

    public Command setPivotShooter(double angle, double speed) {
        return run(() -> {
            shooterSpeed.setDefault(speed);
        }).until(() -> shooterAtSpeed(speed));
    }

    public Command changeKickerState(ShooterState newState) {
        SmartDashboard.putString("Shooter State", newState.name());
        SmartDashboard.putString("Kicker Direction", newState.kickerDirection.name());
        if (newState.kickerDirection == Direction.RUNNING) {
            return runKicker();
        } else if (newState.kickerDirection == Direction.REVERSED) {
            return reverseKicker();
        }

        return stopKicker();
    }

    public Command changeState(ShooterState newState) {
        if (newState == ShooterState.SHOOT_AMP) {
            return setPivotShooter(newState.pivotAngle, 1200);
        } else if (newState == ShooterState.SPIT_OUT) {
            return setPivotShooter(newState.pivotAngle, 1000);
        }

        return setPivotShooter(newState.pivotAngle, newState.runShooter ? Manipulator.SHOOTER_SPEED : 0);
    }
}
