package frc.robot.commands.align;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.StabilizingArm;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.AlignUtil;

public class AlignCommand extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    private final StabilizingArm stabilizingArm; 
    private final AlignUtil alignUtil; 
    private List<Pose2d> currentPoseList;

    public AlignCommand(SwerveSubsystem swerveSubsystem, StabilizingArm stabilizingArm, FieldManagementSubsystem fmsSubsystem){
        this.fmsSubsystem = fmsSubsystem; 
        this.swerveSubsystem = swerveSubsystem;
        this.stabilizingArm = stabilizingArm;
        this.alignUtil = new AlignUtil(swerveSubsystem);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerveSubsystem.getRobotPosition();

        if(fmsSubsystem.isRedAlliance()){
            currentPoseList = AlignConstants.RED_START_POSES;
        }
        else{
            currentPoseList = AlignConstants.BLUE_START_POSES;
        }

        Pose2d closestStartPose = currentPose.nearest(currentPoseList);
        int index = currentPoseList.indexOf(closestStartPose);
        String pathName = AlignConstants.ALIGN_PATHS.get(index); 
        
        if (index>2){
            alignUtil.followPath(pathName);
        }

        else{
            new ClimbSequenceCommand(swerveSubsystem, stabilizingArm, pathName, index);
        }
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
