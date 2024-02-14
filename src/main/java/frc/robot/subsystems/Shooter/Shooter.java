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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.robot.util.Direction;

abstract public class Shooter extends SubsystemBase {
    private final CANSparkMax kicker;
    private final CANSparkMax shooterLeft;
    private final CANSparkMax shooterRight;

    private final PIDTuning tuning;
    private final TunableNumber shooterSpeed;

    private final DigitalInput beamBreak;

    public Shooter() {
        kicker = Manipulator.KICKER_CONFIG.createSparkMax();
        shooterLeft = Manipulator.SHOOTER_CONFIG_LEFT.createSparkMax();
        shooterRight = Manipulator.SHOOTER_CONFIG_RIGHT.createSparkMax();
        shooterLeft.follow(shooterRight, false);
        beamBreak = new DigitalInput(Manipulator.BEAM_BREAK_ID);

        tuning = new PIDTuning("Shooter", Manipulator.SHOOTER_CONFIG_RIGHT.pid, Constants.TUNING_MODE);
        shooterSpeed = new TunableNumber("Shooter Speed", 0, Constants.TUNING_MODE);
    }

    public boolean ringInManipulator() {
        return !beamBreak.get();
    }

    public Command runKicker() {
        return runOnce(() -> kicker.set(Manipulator.KICKER_SPEED));
    }

    public Command reverseKicker() {
        return runOnce(() -> kicker.set(-Manipulator.KICKER_SPEED));
    }

    public Command stopKicker() {
        return runOnce(() -> kicker.set(0));
    }

    public Command runShooter(double speed) {
        return run(() -> {
            shooterRight.getPIDController().setReference(speed,
                    CANSparkBase.ControlType.kVelocity);
        }).until(() -> shooterAtSpeed(speed));
    }

    public boolean shooterAtSpeed(double speed) {
        return (Math.abs(shooterRight.getEncoder().getVelocity() - speed) < Manipulator.SHOOTER_ERROR);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterRight.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter right current", shooterRight.getOutputCurrent());
        SmartDashboard.putNumber("Shooter left current", shooterLeft.getOutputCurrent());
        SmartDashboard.putNumber("Error", shooterLeft.getOutputCurrent());

        if (Constants.TUNING_MODE) {
            tuning.updatePID(shooterRight);
            shooterRight.getPIDController().setReference(shooterSpeed.get(), CANSparkBase.ControlType.kVelocity);
        }
        // shooterRight.set(1);
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

        Command pivotCommand = setPivotPos(newState.pivotAngle); // i trolled

        return pivotCommand.andThen(runShooter(newState.runShooter ? Manipulator.SHOOTER_SPEED : 0))
                .andThen(kickerCommmand);
    }
}
