package frc.robot.commands.shooter.hood;

import frc.robot.subsystems.shooter.hood;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix6.CANBus;

public class hoodCommand extends CommandBase{

    private hood hd;

    public hoodCommand(){
        hd = 
    }

    @Override
    public void execute() {
        hd.setHoodAngle();
    }

    
}
