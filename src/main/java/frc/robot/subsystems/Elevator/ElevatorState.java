package frc.robot.subsystems.Elevator;

public enum ElevatorState {
    IDLE(0),
    SPEAKER(0),
    TRAP(0),
    CLIMB(30),
    AMP(65);

    double position;

    private ElevatorState(double position) {
        this.position = position;
    }

    public double getPos() {
        return position;
    }
}
