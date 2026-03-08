package frc.robot.commands.align;

import edu.wpi.first.math.jni.StateSpaceUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.StabilizingArm;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.AlignUtil;

public class ClimbSequenceCommand extends SequentialCommandGroup {
    private final AlignUtil alignUtil; 
    private  String pathName2;
    static Command climbSequence;
        
    public ClimbSequenceCommand(SwerveSubsystem swerveSubsystem, StabilizingArm stabilizingArm, String pathName, int index){
        alignUtil = new AlignUtil (swerveSubsystem);
        addRequirements(swerveSubsystem, stabilizingArm);
 
        pathName2 = AlignConstants.SECOND_CLIMB_PATHS.get(index);
        addCommands(
            alignUtil.findThenFollowPath(pathName)
            .andThen(stabilizingArm.deployArm(()-> swerveSubsystem.getCurrentCommand()!= null))
            .andThen(alignUtil.followPath(pathName2))
            .andThen(stabilizingArm.retractArm(()->swerveSubsystem.getCurrentCommand()!= null))
        );
    }

}
