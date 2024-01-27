package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Manipulator;

abstract public class Shooter extends SubsystemBase {
    private final CANSparkMax kicker;
    private final CANSparkMax shooterLeft;
    private final CANSparkMax shooterRight;

    private double shooterSpeed;

    public Shooter() {
        kicker = Manipulator.KICKER_CONFIG.createSparkMax();
        shooterLeft = Manipulator.SHOOTER_CONFIG_LEFT.createSparkMax();
        shooterRight = Manipulator.SHOOTER_CONFIG_RIGHT.createSparkMax();
        shooterLeft.follow(shooterRight, false);
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
        return runOnce(() -> {
            shooterSpeed = speed;
            shooterRight.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
        });
    }

    public boolean shooterAtSpeed() {
        return (Math.abs(shooterRight.getEncoder().getVelocity() - shooterSpeed) < 5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterRight.getEncoder().getVelocity());
    }
}
