package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;

public class BetaShooter extends Shooter {
    private final CANSparkMax pivot;

    private TunableNumber pivotPos;

    private final AbsoluteEncoder absoluteEncoder;
    private final ArmFeedforward wristFeedforward;

    private final PIDTuning pivotTuning;

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

        pivotTuning = new PIDTuning("Pivot Motor", Manipulator.PIVOT_CONFIG.pid, Constants.TUNING_MODE);

        absoluteEncoder = new AbsoluteEncoder(Manipulator.ENCODER_ID, Manipulator.ENCODER_OFFSET);
        wristFeedforward = new ArmFeedforward(Manipulator.KS, Manipulator.KG, Manipulator.KV);

        pivotPos = new TunableNumber("Pivot Pose", absoluteEncoder.getPosition(), Constants.TUNING_MODE);
        resetEncoder();
    }

    public void resetEncoder() {
        pivot.getEncoder().setPosition(absoluteEncoder.getPosition());
    }

    public boolean pivotAtPos(double pivotPos) {
        return (Math.abs(pivot.getEncoder().getPosition() - pivotPos) < Manipulator.PIVOT_ERROR);
    }

    @Override
    public Command setPivotPos(double pos) {
        return run(() -> {
            pivotPos.setDefault(pos);
        }).until(() -> pivotAtPos(pos));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Relative Position", pivot.getEncoder().getPosition());

        super.periodic();
        // pivot.getPIDController().setReference(
        //     pivotPos.get(),
        //     CANSparkBase.ControlType.kPosition,
        //     0,
        //     wristFeedforward.calculate(pivot.getEncoder().getPosition(), 0)
        // );

        // if (Constants.TUNING_MODE) {
        //     pivotTuning.updatePID(pivot);
        // }
    }

}