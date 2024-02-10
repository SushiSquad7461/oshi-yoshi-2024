package frc.robot.subsystems.Shooter;

import frc.robot.util.Direction;

public enum ShooterState {
    IDLE(false, Direction.OFF),
    SHOOT(true, Direction.RUNNING),
    FEED(false, Direction.RUNNING),
    REVERSE(false, Direction.REVERSED);

    public boolean runShooter;
    public Direction kickerDirection;

    private ShooterState(boolean runShooter, Direction kickerDirection) {
        this.runShooter = runShooter;
        this.kickerDirection = kickerDirection;
    } 
}
