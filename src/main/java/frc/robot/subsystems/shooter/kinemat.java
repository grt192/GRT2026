package frc.robot.subsystems.shooter;
import frc.robot.Constants.railgunConstants;

public class kinemat {
    
    public static double calculateAngle(double dista){
        return Math.atan((2*3.0/dista)*(1 + Math.sqrt(1 - 1.828/3.0)));
    }

    public static double calculateVel(double ang){
        return Math.sqrt(2*9.8*3.0) / Math.sin(ang);  
    }

    public static double rotationSpeed(double linearVel){
        return linearVel/(Math.PI * railgunConstants.flyDia);
    }

    public static double angleToRot(double ang){
        return ang / 360.0;
    }
}
