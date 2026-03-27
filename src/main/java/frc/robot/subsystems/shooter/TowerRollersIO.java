package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface TowerRollersIO {
    @AutoLog
    public static class TowerRollersIOInputs {
        public double positionRotations = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean connected = false;
    }

    public default void updateInputs(TowerRollersIOInputs inputs) {}

    public default void setDutyCycle(double percentOutput) {}

    public default void setVelocity(double rps) {}

    public default void stop() {}
}
