package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Manipulator;

public class Wrist extends SubsystemBase {
    private final CANSparkMax kicker;
    private final CANSparkMax pivot;
    private final CANSparkMax shooter;

    private double pivotPos;
    private double shooterSpeed;

    private final AbsoluteEncoder absoluteEncoder;
    private final ArmFeedforward wristFeedforward;

    public static Wrist instance;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
            return instance;
        }
        return instance;
    }

    private Wrist() {
        kicker = Manipulator.KICKER_CONFIG.createSparkMax();
        pivot = Manipulator.PIVOT_CONFIG.createSparkMax();
        MotorHelper.setConversionFactor(pivot, Manipulator.PIVOT_GEAR_RATIO);
        shooter = Manipulator.SHOOTER_CONFIG.createSparkMax();

        absoluteEncoder = new AbsoluteEncoder(Manipulator.ENCODER_ID, Manipulator.ENCODER_OFFSET);
        wristFeedforward = new ArmFeedforward(Manipulator.KS, Manipulator.KG, Manipulator.KV);
        pivotPos = absoluteEncoder.getPosition();
        resetEncoder();
    }

    public void resetEncoder() {
        pivot.getEncoder().setPosition(absoluteEncoder.getPosition());
    }

    public boolean shooterAtSpeed() {
        return (Math.abs(shooter.getEncoder().getVelocity() - shooterSpeed) < 5);
    }

    public boolean pivotAtPos() {
        return (Math.abs(absoluteEncoder.getPosition() - pivotPos) < .1);
    }

    public Command setPivotPos(double pos) {
        return runOnce(() -> {
            pivotPos = pos;
        });
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
            shooter.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
        });
    }

    @Override
    public void periodic() {
        pivot.getPIDController().setReference(
                pivotPos,
                CANSparkBase.ControlType.kPosition,
                0,
                wristFeedforward.calculate(pivot.getEncoder().getPosition(), 0));
    }

}