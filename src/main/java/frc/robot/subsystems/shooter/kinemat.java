package frc.robot.subsystems.shooter;
import frc.robot.Constants.railgunConstants;

public class kinemat {
    
    public static double calculateAngle(double dista){
        return Math.toDegrees(Math.atan( (6/(dista))*(1+ Math.sqrt(1- (1.8288/3)) ) ));
    }

    public static double calculateVel(double ang){
        return Math.sqrt(6*9.8/ ((Math.toDegrees(Math.sin(Math.toRadians(ang))))*(Math.toDegrees(Math.sin(Math.toRadians(ang))))) );
    }

    public static double rotationSpeed(double linearVel){
        return linearVel/(Math.PI * railgunConstants.flyDia);
    }

    public static double angleToRot(double ang){
        return ang / 360.0;
    }
}
