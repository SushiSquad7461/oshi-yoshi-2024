package frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
   private static Elevator instance;

   private final TalonFX leftMotor;
   private final TalonFX rightMotor;

   private PIDTuning pid;

   private final TunableNumber setpoint;
   private final ElevatorFeedforward ffd;
   private final ElevatorFeedforward ffu;

   private boolean up;
   private boolean resetElevator;

   public static Elevator getInstance() {
      if (instance == null)
         instance = new Elevator();
      return instance;
   }

   public Elevator() {
      ffd = new ElevatorFeedforward(0, Constants.Elevator.G_DOWN, 0);
      ffu = new ElevatorFeedforward(0, Constants.Elevator.G_UP, 0);
      up = true;

      leftMotor = Constants.Elevator.ELEVATOR_LEFT.createTalon();
      rightMotor = Constants.Elevator.ELEVATOR_RIGHT.createTalon();
      leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

      resetElevator = false;

      if (Constants.TUNING_MODE) {
         pid = new PIDTuning("Elevator",
               new PIDConfig(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D),
               Constants.TUNING_MODE);
      }

      setpoint = new TunableNumber("Elevator Setpoint", 0, Constants.TUNING_MODE);
   }

   public Command changeState(ElevatorState state) {
      return runOnce(
            () -> {
               up = state.getPos() > rightMotor.getPosition().getValueAsDouble();
               setpoint.setDefault(state.getPos());
            }).andThen(new WaitUntilCommand(elevatorInPosition(state.getPos())));
   }

   private BooleanSupplier elevatorInPosition(double elevatorPos) {
      return () -> Math
            .abs(rightMotor.getPosition().getValueAsDouble() - setpoint.get()) < Constants.Elevator.MAX_ERROR;
   }

   public Command resetElevator() {
      return runOnce(() -> {
         rightMotor.set(-0.1);
         resetElevator = true;
      });
   }

   public Command resetElevatorEnd() {
      return runOnce(() -> {
         rightMotor.set(0);
         rightMotor.setPosition(0);
         resetElevator = false;
      });
   }

   @Override
   public void periodic() {
      SmartDashboard.putNumber("Elevator Position", rightMotor.getPosition().getValueAsDouble());

      if (Constants.TUNING_MODE) {
         pid.updatePID(rightMotor);
      }

      if (!resetElevator) {
         rightMotor.setControl(
               new PositionDutyCycle(setpoint.get()).withFeedForward(up ? ffu.calculate(0.0) : ffd.calculate(0.0)));
      }
   }
}
