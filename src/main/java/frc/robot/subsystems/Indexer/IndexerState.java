package frc.robot.subsystems.Indexer;

import frc.robot.util.Direction;

public enum IndexerState {
    IDLE(Direction.OFF),
    REVERSE(Direction.REVERSED),
    INDEX(Direction.RUNNING);

    public Direction direction;

    private IndexerState(Direction direction) {
        this.direction = direction;
    }
}