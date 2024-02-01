package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class BetaIntake extends Intake {
    private final CANSparkMax pivotMotor;

    private static BetaIntake instance;

    private final ArmFeedforward intakeFeedforward;
    private final AbsoluteEncoder absoluteEncoder;

    private double pivotPos;

    public static BetaIntake getInstance() {
        if (instance == null) {
            return new BetaIntake();
        }
        return instance;
    }

    private BetaIntake() {
        super();

        pivotMotor = Constants.Intake.PIVOT_CONFIG.createSparkMax();

        intakeFeedforward = new ArmFeedforward(0.0, Constants.Intake.G, 0.0);
        absoluteEncoder = new AbsoluteEncoder(Constants.Intake.ENCODER_CHANNEL, Constants.Intake.ENCODER_ANGLE_OFFSET);
        MotorHelper.setDegreeConversionFactor(pivotMotor, Constants.Intake.INTAKE_GEAR_RATIO);

        resetToAbsolutePosition();
    }

    public void resetToAbsolutePosition() {
        pivotMotor.getEncoder().setPosition(getPosition());
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getNormalizedPosition();
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < Constants.Intake.MAX_ERROR);
    }

    public double getAbsoluteError() {
        return Math.abs(getPosition() - getAbsolutePosition());
    }

    public double getError(double setpoint) {
        return Math.abs(getPosition() - setpoint);
    }

    public Command setPosition(double pivotPos) {
        return runOnce(() -> {
            this.pivotPos = pivotPos;
        });
    }

    @Override
    public void periodic() {
        if (getAbsoluteError() > Constants.Intake.ERROR_LIMIT) {
            resetToAbsolutePosition();
        }

        pivotMotor.getPIDController().setReference(
            pivotPos,
            ControlType.kPosition,
            0,
            intakeFeedforward.calculate(Math.toRadians(getPosition()), 0.0)
        );

    }
}
