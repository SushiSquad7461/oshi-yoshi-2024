package frc.robot.subsystems.Elevator;

public enum ElevatorState {
    IDLE(0),
    SPEAKER(45),
    TRAP(0),
    CLIMB(20),
    AMP(55);

    double position;

    private ElevatorState(double position) {
        this.position = position;
    }

    public double getPos() {
        return position;
    }
}
