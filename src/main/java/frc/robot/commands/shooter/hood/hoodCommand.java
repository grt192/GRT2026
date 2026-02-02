package frc.robot.commands.shooter.hood;

import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.kinemat;
import com.ctre.phoenix6.CANBus;

public class hoodCommand extends Command{

    private hood hd;
    private flywheel fly;
    private double distance = 0;
    private double angle;
    private double velocity;

    public hoodCommand(hood h, flywheel f){
        hd = h;
        fly = f;
    }

    @Override
    public void execute() {
        angle = kinemat.calculateAngle(distance);
        velocity = kinemat.calculateVel(angle);
        fly.setVelocity(velocity);
        hd.setHoodAngle(angle);
    }

    public void end(){
        fly.setVelocity(0);
        fly.flySpeed(0);
    }
 
}
