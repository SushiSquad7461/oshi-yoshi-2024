package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.robot.util.Direction;

abstract public class Shooter extends SubsystemBase {
    private final CANSparkMax kicker;
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;

    private final PIDTuning tuning;
    private final TunableNumber shooterSpeed;

    // private final DigitalInput beamBreak;

    public Shooter() {
        kicker = Manipulator.KICKER_CONFIG.createSparkMax();
        shooterTop = Manipulator.SHOOTER_CONFIG_TOP.createSparkMax();
        shooterBottom = Manipulator.SHOOTER_CONFIG_BOTTOM.createSparkMax();
        shooterTop.follow(shooterBottom, false);
        // beamBreak = new DigitalInput(Manipulator.BEAM_BREAK_ID);

        tuning = new PIDTuning("Shooter", Manipulator.SHOOTER_CONFIG_BOTTOM.pid, Constants.TUNING_MODE);
        shooterSpeed = new TunableNumber("Shooter Speed", 0, Constants.TUNING_MODE);
    }

    abstract public boolean ringInShooter();

    // public boolean ringInManipulator() {
    // return !beamBreak.get();
    // }

    public Command runKicker() {
        return runOnce(() -> kicker.set(Manipulator.KICKER_SPEED));
    }

    public Command reverseKicker() {
        return runOnce(() -> kicker.set(-0.3));
    }

    public Command stopKicker() {
        return runOnce(() -> kicker.set(0));
    }

    public Command runShooter(double speed) {
        return run(() -> {
            shooterSpeed.setDefault(speed);
        }).until(() -> shooterAtSpeed(speed));
    }

    public boolean shooterAtSpeed(double speed) {
        return (Math.abs(shooterBottom.getEncoder().getVelocity() - speed) < Manipulator.SHOOTER_ERROR);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterBottom.getEncoder().getVelocity());        SmartDashboard.putNumber("Shooter Top", shooterTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("Error", shooterTop.getOutputCurrent());

        tuning.updatePID(shooterBottom);
        shooterBottom.getPIDController().setReference(shooterSpeed.get(), CANSparkBase.ControlType.kVelocity);
    }

    public Command setPivotPos(double angle) {
        return Commands.none();
    }

    public Command changeState(ShooterState newState) {
        Command kickerCommmand;

        if (newState.kickerDirection == Direction.RUNNING) {
            kickerCommmand = runKicker();
        } else if (newState.kickerDirection == Direction.REVERSED) {
            kickerCommmand = reverseKicker();
        } else {
            kickerCommmand = stopKicker();
        }

        Command pivotCommand = setPivotPos(newState.pivotAngle);

        if (newState == ShooterState.SHOOT_AMP) {
            return pivotCommand.andThen(runShooter(1200))
                .andThen(kickerCommmand);
        }

        return pivotCommand.andThen(runShooter(newState.runShooter ? Manipulator.SHOOTER_SPEED : 0))
                .andThen(kickerCommmand);
    }
}
