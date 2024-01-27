package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Manipulator;

public class BetaShooter extends Shooter {
    private final CANSparkMax pivot;

    private double pivotPos;

    private final AbsoluteEncoder absoluteEncoder;
    private final ArmFeedforward wristFeedforward;

    public static BetaShooter instance;

    public static BetaShooter getInstance() {
        if (instance == null) {
            instance = new BetaShooter();
            return instance;
        }
        return instance;
    }

    private BetaShooter() {
        pivot = Manipulator.PIVOT_CONFIG.createSparkMax();
        MotorHelper.setConversionFactor(pivot, Manipulator.PIVOT_GEAR_RATIO);

        absoluteEncoder = new AbsoluteEncoder(Manipulator.ENCODER_ID, Manipulator.ENCODER_OFFSET);
        wristFeedforward = new ArmFeedforward(Manipulator.KS, Manipulator.KG, Manipulator.KV);
        pivotPos = absoluteEncoder.getPosition();
        resetEncoder();
    }

    public void resetEncoder() {
        pivot.getEncoder().setPosition(absoluteEncoder.getPosition());
    }

    public boolean pivotAtPos() {
        return (Math.abs(absoluteEncoder.getPosition() - pivotPos) < .1);
    }

    public Command setPivotPos(double pos) {
        return runOnce(() -> {
            pivotPos = pos;
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