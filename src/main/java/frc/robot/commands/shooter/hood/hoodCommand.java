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
    int n = 0; int s = 0;
    
    public hoodCommand(hood h, flywheel f){
        /*
        SmartDashboard.putNumber("Pos", distance);
        SmartDashboard.putNumber("ShooterHeight", height);
        */
        hd = h;
        fly = f;
        //addRequirements(hd, fly);
    }
    @Override
    public void initialize() {
    System.out.println("hoodCommand INIT");
    }

    @Override
    public boolean runsWhenDisabled() {
    return true; // testing only
    }

    @Override
    public void execute() {
        //System.out.println("Running");
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
        double b = SmartDashboard.getNumber("RPS", s);
        fly.setVelocity(SmartDashboard.getNumber("RPS", s));
        hd.setHoodAngle(SmartDashboard.getNumber("HoodAngle", n));

        // Log for debug
        Logger.recordOutput("hoodCommand/" + "RPSComanded", b);
    }

    @Override
    public void end(boolean interrupted){
        fly.setVelocity(0);
        fly.flySpeed(0);
    }
    
     @Override
    public boolean isFinished() {
        return false; // runs until canceled / trigger released
    }
}
