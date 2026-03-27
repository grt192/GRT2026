package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double positionRotations = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean connected = false;
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setPosition(double rotations) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void stop() {}
}
