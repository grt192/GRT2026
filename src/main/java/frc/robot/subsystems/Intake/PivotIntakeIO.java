package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIntakeIO {
    @AutoLog
    public static class PivotIntakeIOInputs {
        public double motorPositionRotations = 0.0;
        public double motorVelocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public double closedLoopError = 0.0;
        public boolean motorConnected = false;

        public double encoderPositionRotations = 0.0;
        public double encoderAbsolutePosition = 0.0;
        public boolean encoderConnected = false;
    }

    public default void updateInputs(PivotIntakeIOInputs inputs) {}

    public default void setPosition(double rotations) {}

    public default void setManualSpeed(double speed) {}

    public default void stop() {}

    public default void zeroEncoder() {}

    public default void setEncoderToMax() {}
}
