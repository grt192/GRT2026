package frc.robot.commands.shooter.flywheel;

import frc.robot.subsystems.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.kinemat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class rampFlywheel extends Command{
    
    private flywheel fly;
    boolean good;
    
    public rampFlywheel(flywheel h, boolean isGood){
        fly = h;
        good = isGood;
    }

    @Override
    public void execute() {
        double RPS = kinemat.calculateVel();
        fly.shoot(RPS);

        if(fly.getRPS() == RPS){
            good = true;
        }else{good = false;}
    }

    @Override
    public void end(boolean interrupted){
        fly.dontShoot();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
