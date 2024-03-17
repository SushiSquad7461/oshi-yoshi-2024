package frc.robot.subsystems.Elevator;

public enum ElevatorState {
    IDLE(3),
    SPEAKER(3),
    TRAP(3),
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
