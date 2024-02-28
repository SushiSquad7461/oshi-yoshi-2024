package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
   private final TalonFX leftElevator;
   private final TalonFX rightElevator; 

   public Elevator() {
        leftElevator = Constants.Elevator.ELEVATOR_LEFT.createTalon();
        rightElevator = Constants.Elevator.ELEVATOR_RIGHT.createTalon();
   }
}
