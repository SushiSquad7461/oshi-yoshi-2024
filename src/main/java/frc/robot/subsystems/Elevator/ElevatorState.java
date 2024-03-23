package frc.robot.subsystems.Elevator;

public enum ElevatorState {
    IDLE(2),
    SPEAKER(2),
    TRAP(0),
    CLIMB(30),
    AMP(55);

    double position;
    

    private ElevatorState(double position) {
        this.position = position;
    }

    public double getPos() {
        return position;
    }
}
