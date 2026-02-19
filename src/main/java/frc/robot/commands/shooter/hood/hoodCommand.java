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
    private double distance = 0;
    private double angle;
    private double velocity;

    public hoodCommand(hood h, flywheel f){
        hd = h;
        fly = f;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Pos", distance);
        distance = SmartDashboard.getNumber("Pos", distance);
        angle = kinemat.calculateAngle(distance);
        velocity = kinemat.calculateVel(angle);

        fly.setVelocity(kinemat.rotationSpeed(velocity));
        hd.setHoodAngle(-1*(0.25 - kinemat.angleToRot(angle))-0.25);

        Logger.recordOutput("Calculate" + "Expected_Linear_Output",
            velocity);

        Logger.recordOutput("Calculate" + "Expected_RPS", kinemat.rotationSpeed(velocity));
    }

    public void end(){
        fly.setVelocity(0);
        fly.flySpeed(0);
    }
 
}
