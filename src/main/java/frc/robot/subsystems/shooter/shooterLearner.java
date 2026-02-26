package frc.robot.subsystems.shooter;

public class shooterLearner {
    public class ShotLearner {

    private double offset = 0;
    private final double k = 50; // tuning constant

    public double getRPM(double baseRPM){
        return baseRPM + offset;
    }

    public void update(double shotError){
        offset += k * shotError;
    }

    public void reset(){
        offset = 0;
    }
}
}
