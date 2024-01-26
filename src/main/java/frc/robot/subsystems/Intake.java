package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    CANSparkMax intake;
    CANSparkMax uprightRoller;
    public Intake() {
        intake = new CANSparkMax(23, MotorType.kBrushless);
        uprightRoller = new CANSparkMax(26, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        intake.set(0.5);
        uprightRoller.set(0.5);
    }
}
