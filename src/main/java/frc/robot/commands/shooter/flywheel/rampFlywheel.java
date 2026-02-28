package frc.robot.commands.shooter.flywheel;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.shooter.table;

import edu.wpi.first.wpilibj2.command.Command;

public class rampFlywheel extends Command{
    
    private flywheel fly;
    private SwerveSubsystem s;
 
    public rampFlywheel(flywheel h, SwerveSubsystem sw){
        fly = h;
        s = sw;
        addRequirements(fly, sw);
    }

    @Override
    public void execute() {
        double RPS = table.getRPS(s.getDistance());
        fly.shoot(RPS);
    }

    @Override
    public void end(boolean interrupted){
        fly.dontShoot();
    }

}
