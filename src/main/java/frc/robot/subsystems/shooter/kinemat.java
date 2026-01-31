package frc.robot.subsystems.shooter;

public class kinemat {
    
    public static double calculateAngle(double dista){
        return Math.toDegrees(Math.atan( (6/(dista))*(1+ Math.sqrt(1- (1.8288/3)) ) ));
    }

    public static double calculateVel(double ang){
        return Math.sqrt(6*9.8/ ((Math.toDegrees(Math.sin(Math.toRadians(ang))))*(Math.toDegrees(Math.sin(Math.toRadians(ang))))) );
    }
}
