package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double velocityRPS = 0.0;
        public double positionRotations = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean connected = false;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setVelocity(double rps) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void stop() {}
}
