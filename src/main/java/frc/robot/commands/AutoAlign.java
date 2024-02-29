package frc.robot.commands;

import org.apache.commons.collections4.sequence.InsertCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import SushiFrcLib.Swerve.SwerveConstants.SDSModules;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class AutoAlign extends Command{
    private final Swerve swerve;
    private final AlignPosition target;
    private final PathConstraints constraints;

    public enum AlignPosition {
        FENDER(0,0,0, "Fender"),
        STAGE(0,0,0, "Stage"),
        AMP(0,0,0,"Amp");
        Pose2d pos;
        PathPlannerPath path;
        AlignPosition(double x, double y, double rotation, String path) {
            this.pos = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
            this.path = PathPlannerPath.fromPathFile(path);
        }
    }

    public AutoAlign(AlignPosition target) {
        this.target = target;
        swerve =  Swerve.getInstance();
        constraints = new PathConstraints(
            SDSModules.MK4i.maxSpeed, 
            SDSModules.MK4i.maxAcceleration, 
            SDSModules.MK4i.maxAngularVelocity, 
            SDSModules.MK4i.maxAngularAcceleration
        );
    }

    @Override
    public void execute() {
        Command pathFindingCommand = new PathfindThenFollowPathHolonomic(
            target.path,
            constraints,
            swerve::getOdomPose,
            swerve::getChassisSpeeds,
            swerve::driveChassis,
            new HolonomicPathFollowerConfig(
                        Constants.Swerve.AUTO_TRANSLATION,
                        Constants.Swerve.AUTO_ROTATION,
                        Constants.Swerve.MODULE_TYPE.maxSpeed,
                        Constants.Swerve.DRIVE_BASE_RADIUS,
                        new ReplanningConfig()),
            () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
            swerve
        );
        pathFindingCommand.schedule();
    }
}
