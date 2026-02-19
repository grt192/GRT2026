package frc.robot.commands.shooter.hood;

import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.kinemat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class hoodCommand extends Command{

    private hood hd;
    private flywheel fly;
    private double distance = 4.2418;
    private double angle;
    private double velocity;
    private double height = 0;

    int n; int s;
    
    public hoodCommand(hood h, flywheel f){
        /*
        SmartDashboard.putNumber("Pos", distance);
        SmartDashboard.putNumber("ShooterHeight", height);
        */
        SmartDashboard.putNumber("AngleCommanded", n);
        SmartDashboard.putNumber("LinearVelCommanded", s);
        hd = h;
        fly = f;
    }

    @Override
    public void execute() {
        /*
        distance = SmartDashboard.getNumber("Pos", distance);
        double yShooter = SmartDashboard.getNumber("ShooterHeight", height);

        // Compute angle and velocity
        angle = kinemat.calculateAngle(distance, yShooter);
        velocity = kinemat.calculateVel(distance, yShooter);
        

        // Set flywheel and hood
        fly.setVelocity(kinemat.rotationSpeed(velocity));
        hd.setHoodAngle(kinemat.hoodRot(angle) - 0.25);
        Logger.recordOutput("hoodCommand/" + "commandAngle", kinemat.hoodRot(angle) - 0.25);
        */

        fly.setVelocity(kinemat.rotationSpeed(SmartDashboard.getNumber("LinearVelCommanded", s)));
        hd.setHoodAngle(-1*SmartDashboard.getNumber("AngleCommanded", n)/360 - 0.25);

        // Log for debug
        Logger.recordOutput("hoodCommand/" + "Expected_Linear_Output", velocity);
        Logger.recordOutput("hoodCommand/" + "Expected_RPS", kinemat.rotationSpeed(velocity));
    }

    public void end(){
        fly.setVelocity(0);
        fly.flySpeed(0);
    }
 
}
