package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
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

    public static DigitalInput beamBreak;

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
        MotorHelper.setDegreeConversionFactor(pivot, Manipulator.PIVOT_GEAR_RATIO);

        absoluteEncoder = new AbsoluteEncoder(Manipulator.ENCODER_ID, Manipulator.ENCODER_OFFSET, false);
        wristFeedforward = new ArmFeedforward(Manipulator.KS, Manipulator.KG, Manipulator.KV);

        pivotTuning =  Manipulator.PIVOT_CONFIG.genPIDTuning("Pivot Shooter", Constants.TUNING_MODE);
        pivotPos = new TunableNumber("Pivot Pose", Manipulator.PIVOT_IDLE, Constants.TUNING_MODE);
        beamBreak = new DigitalInput(3);
        resetEncoder();
        pivot.getEncoder().setPosition(-59.8);
    }

    public void resetEncoder() {
        pivot.getEncoder().setPosition(absoluteEncoder.getPosition());
    }

    public boolean pivotAtPos(double pivotPos) {
        return (Math.abs(pivot.getEncoder().getPosition() - pivotPos) < Manipulator.PIVOT_ERROR);
    }

    public boolean ringInShooter() {
        return !beamBreak.get();
    }

    @Override
    public Command setPivotPos(double pos) {
        return run(() -> {
            pivotPos.setDefault(pos);
        }).until(() -> pivotAtPos(pos));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Absolute Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Relative Position", pivot.getEncoder().getPosition());
        SmartDashboard.putBoolean("Shooter in ring", ringInShooter());

        super.periodic();

        pivot.getPIDController().setReference(
            pivotPos.get(),
            CANSparkBase.ControlType.kPosition,
            0,
            wristFeedforward.calculate(Math.toRadians(pivot.getEncoder().getPosition()), 0)
        );

        pivotTuning.updatePID(pivot);
     }

}