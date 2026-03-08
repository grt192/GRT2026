    
package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.AlignConstants;
import frc.robot.commands.swerve.DriveBackwardsCommand;

public class AlignUtil {

    private SwerveSubsystem swerveSubsystem;
    private PathPlannerPath getAlignPath;
    private PathConstraints constraints;
    static Command runAlignPath;

    public AlignUtil(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.constraints = new PathConstraints(
            4.6,
            3,
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720)
        );

    }
   
//     /**
//      * Uses getAlignPath to get the pathplanner path and follows it
//      * @param swerveSubsystem
//      * @param pathName name of the path
//      */

    public Command followPath(String pathName){
        PathPlannerPath path = GetPath.getAlignPath(pathName);
        if (path == null) {
                System.out.println("NOOOOOOOOPOOOOOOO");
                return Commands.none();
            }
        System.out.println("YYYYYY");
        Command followPath = AutoBuilder.followPath(path);  
           
        return followPath;
    }

    public Command findThenFollowPath (String pathName) {
        Translation2d currentTrans = swerveSubsystem.getRobotPosition().getTranslation();
        Translation2d pathStartTrans = GetPath.getAlignPath(pathName).getStartingHolonomicPose().get().getTranslation();
        PathPlannerPath path = GetPath.getAlignPath(pathName);

        if (Math.abs(currentTrans.getDistance(pathStartTrans)) <= AlignConstants.DISTANCE_TOLERANCE) {
            if (path == null) {
                System.out.println("NOOO00000000000000O");
                return Commands.none();

            }
            Command alignPath = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);

            System.out.println("YYYYYYYY");
            alignPath.addRequirements(swerveSubsystem);

            runAlignPath = (Command) new SequentialCommandGroup(
                new DriveBackwardsCommand(swerveSubsystem, AlignConstants.DRIVE_POWER).until(
                    () -> Math.abs(swerveSubsystem.getRobotPosition().getTranslation()
                    .getDistance(pathStartTrans)) > AlignConstants.DISTANCE_TOLERANCE),
                    alignPath
            );

        }
        else {
            if (path == null) {
                System.out.println("NOOOOOOOOPOOOOOOO");
                return Commands.none();
            }

        System.out.println("YYYYYYYY");
        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints);  
        }

        return runAlignPath; 

    }
}
