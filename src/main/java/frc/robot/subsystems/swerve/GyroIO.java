package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yawDegrees = 0.0;
        public double pitchDegrees = 0.0;
        public double rollDegrees = 0.0;
        public double yawVelocityDegPerSec = 0.0;
        public boolean connected = false;
        public boolean stickyFaultUndervoltage = false;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void reset() {}
}
