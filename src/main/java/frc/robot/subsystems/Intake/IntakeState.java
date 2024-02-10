package frc.robot.subsystems.Intake;

import frc.robot.util.Direction;

public enum IntakeState {
    IDLE(false, Direction.OFF),
    INTAKE(true, Direction.RUNNING),
    REVERSE(true, Direction.REVERSED);

    public boolean intakeExtended;
    public Direction direction;

    private IntakeState(boolean extended, Direction direction) {
        this.intakeExtended = extended;
        this.direction = direction;
    }
} 