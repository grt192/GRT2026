package frc.robot.commands.shooter.hood;

import frc.robot.subsystems.shooter.hood;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.kinemat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class hoodCommand extends Command{

    private hood hd;
    boolean good;
    
    public hoodCommand(hood h, boolean isGood){
        hd = h;
        good = isGood;
    }

    @Override
    public void execute() {
        double ang = kinemat.calculateAngle();
        hd.setHoodAngle(ang);

        if(hd.getPos() == ang){
            good = true;
        }else{good = false;}
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
 
}
