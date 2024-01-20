package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class Intake extends SubsystemBase {
    private final CANSparkMax indexerMotor;
    private final CANSparkMax rollerMotor;
    private final CANSparkMax pivotMotor;
    private static Intake instance;
    private final ArmFeedforward intakeFeedforward;
    private final AbsoluteEncoder absoluteEncoder;
    private double pivotPos;

    public static Intake getInstance() {
        if (instance == null) {
            return new Intake();
        }
        return instance;
    }

    private Intake() {
        indexerMotor = Constants.Intake.INTAKE_INDEXER_CONFIG.createSparkMax();
        rollerMotor = Constants.Intake.INTAKE_UPRIGHT_ROLLERS_CONFIG.createSparkMax();
        pivotMotor = Constants.Intake.INTAKE_PIVOT_CONFIG.createSparkMax();
        intakeFeedforward = new ArmFeedforward(0.0, Constants.Intake.G, 0.0);
        absoluteEncoder = new AbsoluteEncoder(Constants.Intake.ENCODER_CHANNEL, Constants.Intake.ENCODER_ANGLE_OFFSET);
        MotorHelper.setDegreeConversionFactor(pivotMotor, Constants.Intake.INTAKE_GEAR_RATIO);
    }

    public void resetPosition() {
        pivotMotor.getEncoder().setPosition(getPosition());
    }

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public BooleanSupplier closeToSetpoint(double setpoint) {
        return () -> (getError(setpoint) < Constants.Intake.MAX_ERROR);
    }

    public double getAbsoluteError() {
        return Math.abs(getPosition() - absoluteEncoder.getNormalizedPosition());
    }

    public double getError(double setpoint) {
        return Math.abs(getPosition() - setpoint);
    }

    public Command setPosition(double pivotPos) {
        return runOnce(() -> {
            this.pivotPos = pivotPos;
        });
    }

    public Command runMotor() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Intake.SPIN_SPEED);
            rollerMotor.set(Constants.Intake.SPIN_SPEED);
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            indexerMotor.set(0);
            rollerMotor.set(0);
        });
    }

    public Command reverseMotor() {
        return runOnce(() -> {
            indexerMotor.set(Constants.Intake.SPIN_SPEED * -1);
            rollerMotor.set(Constants.Intake.SPIN_SPEED * -1);
        });
    }

    @Override
    public void periodic() {
        if (getAbsoluteError() > Constants.Intake.ERROR_LIMIT) {
            resetPosition();
        }
        pivotMotor.getPIDController().setReference(
                pivotPos,
                ControlType.kPosition,
                0,
                intakeFeedforward.calculate(Math.toRadians(getPosition()), 0.0));

    }
}
