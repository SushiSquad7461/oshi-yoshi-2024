package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class BetaIntake extends Intake {
    private final CANSparkMax pivotMotor;

    private static BetaIntake instance;

    private final ArmFeedforward intakeFeedforward;
    private final AbsoluteEncoder absoluteEncoder;

    private TunableNumber pivotPos;

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

        pivotPos = new TunableNumber("Intake Pos", getAbsolutePosition(), Constants.TUNING_MODE);
    }

    public void resetToAbsolutePosition() {
        pivotMotor.getEncoder().setPosition(getAbsolutePosition());
    }

    public double getPosition() {
        return pivotMotor.getEncoder().getPosition();
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

    @Override
    public Command lowerIntake() {
        return changePivotPos(Constants.Intake.LOWERED_POS);
    }

    @Override
    public Command raiseIntake() {
        return changePivotPos(Constants.Intake.RAISED_POS);
    }

    public Command changePivotPos(double position) {
        return run(() -> {
            pivotPos.setDefault(position);
        }).until(() -> getError(position) < Constants.Intake.MAX_ERROR);
    }

    @Override
    public void periodic() {
        if (getAbsoluteError() > Constants.Intake.ERROR_LIMIT) {
            resetToAbsolutePosition();
        }

        SmartDashboard.putNumber("Absolute Encoder", getAbsolutePosition());
        SmartDashboard.putNumber("Relative Encoder", pivotMotor.getEncoder().getPosition());

        super.periodic();

        //intakeMotor.set(0.1);

        pivotMotor.getPIDController().setReference(
            pivotPos.get(),
            ControlType.kPosition,
        0,
            intakeFeedforward.calculate(Math.toRadians(getPosition()), 0.0)
        );
    }
}
