package frc.robot.subsystems;

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
    private final CANSparkMax spin1Motor;
    private final CANSparkMax spin2Motor;
    private final CANSparkMax pivotMotor;
    private static Intake instance;
    private final ArmFeedforward intakeFeedforward;
    private final AbsoluteEncoder absoluteEncoder;
    private final double currentPos;
    private MotorConfig intakePID;

    private double pivotPos;

    public static Intake getInstance() {
        if (instance == null) {
            return new Intake();
        }
        return instance;
    }

    private Intake() {
        spin1Motor = Constants.Intake.INTAKE_SPIN1_CONFIG.createSparkMax();
        spin2Motor = Constants.Intake.INTAKE_SPIN2_CONFIG.createSparkMax();
        pivotMotor = Constants.Intake.INTAKE_PIVOT_CONFIG.createSparkMax();
        intakeFeedforward = new ArmFeedforward(0.0, Constants.Intake.G, 0.0);
        absoluteEncoder = new AbsoluteEncoder(Constants.Intake.ENCODER_CHANNEL, Constants.Intake.ENCODER_ANGLE_OFFSET);
        MotorHelper.setDegreeConversionFactor(pivotMotor, Constants.Intake.INTAKE_GEAR_RATIO);
        MotorHelper.setConversionFactor(pivotMotor, Constants.Intake.INTAKE_GEAR_RATIO);
        intakePID = Constants.Intake.INTAKE_PIVOT_CONFIG;
        currentPos = absoluteEncoder.getPosition();
    }

    public Command setPosition(double pivotPos) {
        return runOnce(() -> {
            this.pivotPos = pivotPos;
        });
    }

    public Command runMotor() {
        return runOnce(() -> {
            spin1Motor.set(Constants.Intake.SPIN_SPEED);
            spin2Motor.set(Constants.Intake.SPIN_SPEED);
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            spin1Motor.set(0);
            spin2Motor.set(0);
        });
    }

    @Override
    public void periodic() {
        pivotMotor.getPIDController().setReference(
                pivotPos,
                ControlType.kPosition,
                0,
                intakeFeedforward.calculate(Math.toRadians(currentPos), 0.0));
    }
}
