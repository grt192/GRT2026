package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMPS = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveTorqueCurrentAmps = 0.0;
        public double driveTemperatureCelsius = 0.0;
        public double driveTargetRPS = 0.0;

        public double steerPositionRotations = 0.0;
        public double steerVelocityRPM = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerSupplyCurrentAmps = 0.0;
        public double steerTorqueCurrentAmps = 0.0;
        public double steerTemperatureCelsius = 0.0;
        public double steerClosedLoopError = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVelocity(double metersPerSec) {}

    public default void setSteerPosition(double radians) {}

    public default void setSteerCruiseVelocity(double velocity) {}

    public default void configureDrivePID(double p, double i, double d, double s, double v) {}

    public default void configureSteerPID(double p, double i, double d, double s) {}
}
