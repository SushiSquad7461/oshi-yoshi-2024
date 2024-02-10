package frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;

import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private Elevator instance;
    private CANSparkMax leftElevator;
    private CANSparkMax rightElevator;

    private PIDTuning pid;

    private final TunableNumber setpoint;
    private final ElevatorFeedforward ffd;
    private final ElevatorFeedforward ffu;
    private static boolean up;

    private boolean resetElevator;

    public Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    private Elevator() {
        ffd = new ElevatorFeedforward(0, Constants.Elevator.G_DOWN, 0);
        ffu = new ElevatorFeedforward(0, Constants.Elevator.G_UP, 0);
        up = true;

        leftElevator = Constants.Elevator.LEFT_ELEVATOR.createSparkMax();
        rightElevator = Constants.Elevator.RIGHT_ELEVATOR.createSparkMax();

        leftElevator.follow(rightElevator);

        resetElevator = false;

        if (Constants.TUNING_MODE) {
            pid = new PIDTuning("Elevator", Constants.Elevator.P_UP, Constants.Elevator.I, Constants.Elevator.D, 0,
                    Constants.TUNING_MODE);
        }

        setpoint = new TunableNumber("Elevator Setpoint", 0, Constants.TUNING_MODE);
    }

    public Command changeState(ElevatorState state) {
        return new SequentialCommandGroup(
                runOnce(
                        () -> {
                            up = state.getPos() > rightElevator.getEncoder().getPosition();
                            rightElevator.getPIDController()
                                    .setP(up ? Constants.Elevator.P_UP : Constants.Elevator.P_DOWN);
                            setpoint.setDefault(state.getPos());
                        }),
                new WaitUntilCommand(elevatorInPosition(state.getPos())));
    }

    private BooleanSupplier elevatorInPosition(double elevatorPos) {
        return () -> Math.abs(rightElevator.getEncoder().getPosition() - setpoint.get()) < Constants.Elevator.MAX_ERROR;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", rightElevator.getEncoder().getPosition());

        if (Constants.TUNING_MODE) {
            pid.updatePID(rightElevator);
        }

        if (!resetElevator) {
            rightElevator.getPIDController().setReference(
                    setpoint.get(),
                    CANSparkMax.ControlType.kPosition,
                    0,
                    up ? ffu.calculate(0.0) : ffd.calculate(0.0));
        }
    }

}
