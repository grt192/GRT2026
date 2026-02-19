package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;

public class kinemat {
    private static final double g = 9.8, yTarget = 1.83, yApex = 3.0;

    // Launch angle (radians) given horizontal distance x and shooter height yShooter
    public static double calculateAngle(double x, double yShooter){
        double vY = Math.sqrt(2 * g * (yApex - yShooter));
        double t = Math.max((-vY + Math.sqrt(vY*vY - 2*g*(yTarget - yShooter)))/(g), 
                            (-vY - Math.sqrt(vY*vY - 2*g*(yTarget - yShooter)))/(g));
        return Math.atan2(vY, x/t);
    }

    // Exit velocity (m/s)
    public static double calculateVel(double x, double yShooter){
        double angle = calculateAngle(x, yShooter), vY = Math.sqrt(2*g*(yApex - yShooter));
        double t = Math.max((-vY + Math.sqrt(vY*vY - 2*g*(yTarget - yShooter)))/(g), 
                            (-vY - Math.sqrt(vY*vY - 2*g*(yTarget - yShooter)))/(g));
        return Math.sqrt(Math.pow(x/t, 2) + vY*vY);
    }

    // Flywheel RPS
    public static double rotationSpeed(double linearVel) { return linearVel / (Math.PI * railgunConstants.flyDia); }

    // Radians to rotations
    public static double angleToRot(double rad) { return rad / (2 * Math.PI); }

    // Hood rotation (- = down, 0 = upright)
    public static double hoodRot(double rad) { return -angleToRot(Math.PI/2 - rad); }
}
