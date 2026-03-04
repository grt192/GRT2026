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

public class ClimbSequenceCommand extends Command {
    private final AlignUtil alignUtil; 
    private final SwerveSubsystem swerveSubsystem;
    private final StabilizingArm stabilizingArm;
    private final String pathName;
    private static int index;
    private  String pathName2;
    static Command climbSequence;
        
    public ClimbSequenceCommand(SwerveSubsystem swerveSubsystem, StabilizingArm stabilizingArm, String pathName, int index){
        this.swerveSubsystem = swerveSubsystem;
        this.stabilizingArm = stabilizingArm;
        this.alignUtil = new AlignUtil(swerveSubsystem);
        this.pathName = pathName;
        this.index = index;
        addRequirements(swerveSubsystem, stabilizingArm);
    }
    
    public Command climbSequence(){
        pathName2 = AlignConstants.SECOND_CLIMB_PATHS.get(index);
        climbSequence = (Command) new SequentialCommandGroup(
            alignUtil.findThenFollowPath(pathName)
            .andThen(stabilizingArm.deployArm(()-> swerveSubsystem.getCurrentCommand()!= null))
            .andThen(alignUtil.followPath(pathName2))
            .andThen(stabilizingArm.retractArm(()->swerveSubsystem.getCurrentCommand()!= null))
        );
        return climbSequence;
    }

}
