package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIntakeIO {
    @AutoLog
    public static class RollerIntakeIOInputs {
        public double positionRotations = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double dutyCycle = 0.0;
        public boolean connected = false;
    }

    public default void updateInputs(RollerIntakeIOInputs inputs) {}

    public default void setVelocity(double velocity) {}

    public default void setDutyCycle(double dutyCycle) {}

    public default void stop() {}
}
