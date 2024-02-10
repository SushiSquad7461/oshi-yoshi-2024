package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.Angle;
import frc.robot.Constants;
import frc.robot.util.Direction;

public enum ShooterState {
    IDLE(false, Direction.OFF, Constants.Manipulator.PIVOT_IDLE),
    SHOOT_ANYWHERE(true, Direction.RUNNING, Constants.Manipulator.PIVOT_ANY_ANGLE), // figure out this
    SHOOT_FENDOR(true, Direction.RUNNING, Constants.Manipulator.PIVOT_FENDOR_ANGLE),
    SHOOT_AMP(true, Direction.RUNNING, Constants.Manipulator.PIVOT_AMP_ANGLE),
    FEED(false, Direction.RUNNING, Constants.Manipulator.PIVOT_IDLE),
    REVERSE(false, Direction.REVERSED, Constants.Manipulator.PIVOT_IDLE);

    public boolean runShooter;
    public Direction kickerDirection;
    public double pivotAngle;

    private ShooterState(boolean runShooter, Direction kickerDirection, double pivotAngle) {
        this.runShooter = runShooter;
        this.kickerDirection = kickerDirection;
        this.pivotAngle = pivotAngle;
    }
}
