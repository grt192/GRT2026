package frc.robot.commands.tower;

import frc.robot.subsystems.shooter.towerRollers;
import edu.wpi.first.wpilibj2.command.Command;

public class towerRoll extends Command{
    
    private towerRollers t;

    public towerRoll(towerRollers te){
        t = te;
        addRequirements(t);
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
