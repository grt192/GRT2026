package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class KrakenSwerveModule {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final String name;
    private final double offsetRads;

    public KrakenSwerveModule(String name, ModuleIO io, double offsetRads) {
        this.name = name;
        this.io = io;
        this.offsetRads = offsetRads;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + name, inputs);
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = getWrappedAngle();
        state.optimize(currentAngle);

        double targetAngleRads = state.angle.getRadians() - offsetRads;
        double angleErrorRads = state.angle.minus(currentAngle).getRadians();
        double targetVelocity = state.speedMetersPerSecond * Math.cos(angleErrorRads);

        io.setDriveVelocity(targetVelocity);
        io.setSteerPosition(targetAngleRads);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, getWrappedAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocityMPS, getWrappedAngle());
    }

    public Rotation2d getWrappedAngle() {
        double angleRads = (2.0 * Math.PI * inputs.steerPositionRotations) - Math.PI;
        angleRads = MathUtil.angleModulus(angleRads + offsetRads);
        return new Rotation2d(angleRads);
    }

    public double getSteerVelocityRPM() {
        return inputs.steerVelocityRPM;
    }

    public void setSteerCruiseVelocity(double velocity) {
        io.setSteerCruiseVelocity(velocity);
    }

    public void configureDrivePID(double p, double i, double d, double s, double v) {
        io.configureDrivePID(p, i, d, s, v);
    }

    public void configureSteerPID(double p, double i, double d, double s) {
        io.configureSteerPID(p, i, d, s);
    }
}
