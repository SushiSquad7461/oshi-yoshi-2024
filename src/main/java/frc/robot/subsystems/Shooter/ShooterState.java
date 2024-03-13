package frc.robot.subsystems.Shooter;

import frc.robot.Constants.Manipulator;
import frc.robot.util.Direction;

public enum ShooterState {
    IDLE(false, Direction.OFF, Manipulator.PIVOT_IDLE),
    SHOOT_ANYWHERE(true, Direction.RUNNING, Manipulator.PIVOT_IDLE), // figure out this
    SHOOT_FENDOR(true, Direction.RUNNING, Manipulator.PIVOT_IDLE), //60
    SHOOT_AMP(true, Direction.RUNNING, Manipulator.PIVOT_AMP_ANGLE),
    SHOOT_TRAP(true, Direction.RUNNING, Manipulator.PIVOT_TRAP_ANGLE),
    SHOOT_STAGE(true, Direction.RUNNING, Manipulator.PIVOT_STAGE_ANGLE),
    SPIT_OUT(true, Direction.RUNNING, Manipulator.PIVOT_IDLE ),
    FEED(false, Direction.RUNNING, Manipulator.PIVOT_IDLE),
    REVERSE(false, Direction.REVERSED, Manipulator.PIVOT_IDLE);

    public boolean runShooter;
    public Direction kickerDirection;
    public double pivotAngle;

    private ShooterState(boolean runShooter, Direction kickerDirection, double pivotAngle) {
        this.runShooter = runShooter;
        this.kickerDirection = kickerDirection;
        this.pivotAngle = pivotAngle;
    }
}
