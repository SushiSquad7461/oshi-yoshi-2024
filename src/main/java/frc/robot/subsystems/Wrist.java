package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

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
        shooter = Manipulator.SHOOTER_CONFIG.createSparkMax();
        absoluteEncoder = new AbsoluteEncoder(Manipulator.ENCODER_ID, Manipulator.ENCODER_OFFSET);
        wristFeedforward = new ArmFeedforward(Manipulator.KS, Manipulator.KG, Manipulator.KV);
    }

    public void setPivotPos(double pos) {
        pivotPos = pos;
    }

    public Command runKicker() {
        return runOnce(() -> {
            kicker.set(Manipulator.KICKER_SPEED);
        });
    }

    public Command stopKicker() {
        return runOnce(() -> {
            kicker.set(0);
        });
    }

    public Command runShooter(double speed) {
        return runOnce(() -> {
            shooter.getPIDController().setReference(speed, CANSparkBase.ControlType.kCurrent);
        });
    }

    @Override
    public void periodic() {
        pivot.set(pivotPos);
    }

}