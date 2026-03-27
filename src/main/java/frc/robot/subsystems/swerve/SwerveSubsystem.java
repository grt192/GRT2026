package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.SwerveSteerConstants.STEER_CRUISE_VELOCITY;
import static frc.robot.Constants.SwerveSteerConstants.STEER_GEAR_REDUCTION;

import frc.robot.subsystems.Vision.TimestampedVisionUpdate;

public class SwerveSubsystem extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final KrakenSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private final KrakenSwerveModule[] modules;

    private SwerveModuleState[] states = {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
    };

    private Pose2d estimatedPose = new Pose2d(10, 4, new Rotation2d());
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d driverHeadingOffset = new Rotation2d();

    private Timer lockTimer;
    private double currentCruiseVelocityRPM = STEER_CRUISE_VELOCITY * STEER_GEAR_REDUCTION * 60.0;

    private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    private double lastUpdateTime = 0.0;

    public double maxLinearAcceleration = MAX_LINEAR_ACCELERATION;
    public double maxLinearDeceleration = MAX_LINEAR_DECELERATION;
    public double maxAngularAcceleration = MAX_ANGULAR_ACCELERATION;
    public double maxAngularDeceleration = MAX_ANGULAR_DECELERATION;

    private boolean boostModeEnabled = false;
    private double driveSpeedLimit = 1.0;

    private final PIDController ROTATION_PID = new PIDController(4.0, 0.0, 0.2);

    public SwerveSubsystem(GyroIO gyroIO, ModuleIO flIO, ModuleIO frIO, ModuleIO blIO, ModuleIO brIO) {
        this.gyroIO = gyroIO;
        ROTATION_PID.enableContinuousInput(-Math.PI, Math.PI);

        frontLeftModule = new KrakenSwerveModule("FL", flIO, FL_OFFSET);
        frontRightModule = new KrakenSwerveModule("FR", frIO, FR_OFFSET);
        backLeftModule = new KrakenSwerveModule("BL", blIO, BL_OFFSET);
        backRightModule = new KrakenSwerveModule("BR", brIO, BR_OFFSET);
        modules = new KrakenSwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, getGyroHeading(), getModulePositions(), new Pose2d());

        buildAuton();
        lockTimer = new Timer();
    }

    @Override
    public void periodic() {
        // Update IO
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (KrakenSwerveModule module : modules) {
            module.periodic();
        }

        // Update pose
        Rotation2d gyroAngle = getGyroHeading();
        estimatedPose = poseEstimator.update(gyroAngle, getModulePositions());

        // Idle detection and lock
        boolean isIdle = states[0].speedMetersPerSecond == 0.0
            && states[1].speedMetersPerSecond == 0.0
            && states[2].speedMetersPerSecond == 0.0
            && states[3].speedMetersPerSecond == 0.0;

        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        if (lockTimer.hasElapsed(1)) {
            applyLock();
        } else {
            frontLeftModule.setDesiredState(states[0]);
            frontRightModule.setDesiredState(states[1]);
            backLeftModule.setDesiredState(states[2]);
            backRightModule.setDesiredState(states[3]);
        }

        // Log outputs
        Logger.recordOutput("Swerve/EstimatedPose", estimatedPose);
        Logger.recordOutput("Swerve/GyroHeadingDeg", gyroAngle.getDegrees());
        Logger.recordOutput("Swerve/ModuleStates", getModuleStates());
        Logger.recordOutput("Swerve/BoostMode", boostModeEnabled);
        Logger.recordOutput("Swerve/SpeedLimit", driveSpeedLimit);

        ChassisSpeeds speeds = getRobotRelativeChassisSpeeds();
        Logger.recordOutput("Swerve/ChassisVxMPS", speeds.vxMetersPerSecond);
        Logger.recordOutput("Swerve/ChassisVyMPS", speeds.vyMetersPerSecond);
        Logger.recordOutput("Swerve/ChassisOmegaRadPS", speeds.omegaRadiansPerSecond);
    }

    public void setDrivePowers(double xPower, double yPower, double angularPower) {
        double baseMaxVel = boostModeEnabled ? BOOST_MAX_VEL : MAX_VEL;
        double baseMaxOmega = boostModeEnabled ? (BOOST_MAX_VEL / FL_POS.getNorm()) : MAX_OMEGA;

        double limitedMaxVel = baseMaxVel * driveSpeedLimit;
        double limitedMaxOmega = baseMaxOmega * driveSpeedLimit;

        Rotation2d heading = getDriverHeading();

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * limitedMaxVel, yPower * limitedMaxVel,
            angularPower * limitedMaxOmega, heading);

        ChassisSpeeds speeds = limitAcceleration(desiredSpeeds);

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds, limitedMaxVel, limitedMaxVel, limitedMaxOmega);
    }

    public void setBoostMode(boolean enabled) {
        this.boostModeEnabled = enabled;
    }

    public boolean isBoostModeEnabled() {
        return boostModeEnabled;
    }

    private ChassisSpeeds limitAcceleration(ChassisSpeeds desiredSpeeds) {
        if (boostModeEnabled) {
            maxLinearAcceleration = BOOST_MAX_LINEAR_ACCELERATION;
            maxLinearDeceleration = MAX_LINEAR_DECELERATION;
            maxAngularAcceleration = BOOST_MAX_ANGULAR_ACCELERATION;
            maxAngularDeceleration = MAX_ANGULAR_DECELERATION;
        } else {
            maxLinearAcceleration = MAX_LINEAR_ACCELERATION;
            maxLinearDeceleration = MAX_LINEAR_DECELERATION;
            maxAngularAcceleration = MAX_ANGULAR_ACCELERATION;
            maxAngularDeceleration = MAX_ANGULAR_DECELERATION;
        }

        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;

        if (lastUpdateTime == 0.0 || dt > 0.1 || dt <= 0.0) {
            lastUpdateTime = currentTime;
            previousSpeeds = desiredSpeeds;
            return desiredSpeeds;
        }

        double currentLinearVel = Math.hypot(previousSpeeds.vxMetersPerSecond, previousSpeeds.vyMetersPerSecond);
        double desiredLinearVel = Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);

        double vx = desiredSpeeds.vxMetersPerSecond;
        double vy = desiredSpeeds.vyMetersPerSecond;
        double deltaV = desiredLinearVel - currentLinearVel;

        if (deltaV > 0) {
            double maxDeltaV = maxLinearAcceleration * dt;
            if (deltaV > maxDeltaV) {
                double limitedLinearVel = currentLinearVel + maxDeltaV;
                double scale = limitedLinearVel / desiredLinearVel;
                vx = desiredSpeeds.vxMetersPerSecond * scale;
                vy = desiredSpeeds.vyMetersPerSecond * scale;
            }
        } else if (deltaV < 0) {
            double maxDeltaV = maxLinearDeceleration * dt;
            if (-deltaV > maxDeltaV) {
                double limitedLinearVel = currentLinearVel - maxDeltaV;
                if (desiredLinearVel > 0.001) {
                    double scale = limitedLinearVel / desiredLinearVel;
                    vx = desiredSpeeds.vxMetersPerSecond * scale;
                    vy = desiredSpeeds.vyMetersPerSecond * scale;
                } else {
                    double prevMag = Math.hypot(previousSpeeds.vxMetersPerSecond, previousSpeeds.vyMetersPerSecond);
                    if (prevMag > 0.001) {
                        vx = (previousSpeeds.vxMetersPerSecond / prevMag) * limitedLinearVel;
                        vy = (previousSpeeds.vyMetersPerSecond / prevMag) * limitedLinearVel;
                    }
                }
            }
        }

        double omega = desiredSpeeds.omegaRadiansPerSecond;
        double deltaOmega = omega - previousSpeeds.omegaRadiansPerSecond;
        boolean isAngularAccelerating = Math.abs(omega) > Math.abs(previousSpeeds.omegaRadiansPerSecond);
        double maxDeltaOmega = (isAngularAccelerating ? maxAngularAcceleration : maxAngularDeceleration) * dt;

        if (Math.abs(deltaOmega) > maxDeltaOmega) {
            omega = previousSpeeds.omegaRadiansPerSecond + Math.signum(deltaOmega) * maxDeltaOmega;
        }

        ChassisSpeeds limitedSpeeds = new ChassisSpeeds(vx, vy, omega);
        lastUpdateTime = currentTime;
        previousSpeeds = limitedSpeeds;
        return limitedSpeeds;
    }

    public void applyLock() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
    }

    public void addVisionMeasurements(TimestampedVisionUpdate update) {
        poseEstimator.addVisionMeasurement(update.pose(), update.timestamp(), update.stdDevs());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(),
                backLeftModule.getPosition(), backRightModule.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeftModule.getState(), frontRightModule.getState(),
                backLeftModule.getState(), backRightModule.getState()
        };
    }

    public Rotation2d getDriverHeading() {
        return getGyroHeading().minus(driverHeadingOffset);
    }

    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
    }

    public void resetDriverHeadingOffset90() {
        resetDriverHeading(Rotation2d.fromDegrees(90));
    }

    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(gyroInputs.yawDegrees);
    }

    public Pose2d getRobotPosition() {
        return estimatedPose;
    }

    public void resetPose(Pose2d currentPose) {
        Rotation2d gyroAngle = getGyroHeading().times(-1);
        poseEstimator.resetPosition(gyroAngle, getModulePositions(), currentPose);
    }

    public void resetToStartingPosition() {
        Pose2d startingPose = new Pose2d(10.4, 4.0, new Rotation2d());
        resetPose(startingPose);
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        SwerveModuleState[] currentModuleStates = getModuleStates();
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            kinematics.toChassisSpeeds(currentModuleStates),
            getRobotPosition().getRotation());
    }

    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, new Rotation2d(0));
        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds, MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    public void setRobotRelativeDrivePowers(double xPower, double yPower, double angularPower) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xPower * MAX_VEL, yPower * MAX_VEL,
            angularPower * MAX_OMEGA, new Rotation2d(0));
        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds, MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    public void setDriveSpeedLimit(double limit) {
        this.driveSpeedLimit = Math.max(0.0, Math.min(1.0, limit));
    }

    public double getDriveSpeedLimit() {
        return driveSpeedLimit;
    }

    public void setSteerSpeedLimit(double limit) {
        double velocity = STEER_CRUISE_VELOCITY * limit;
        currentCruiseVelocityRPM = velocity * STEER_GEAR_REDUCTION * 60.0;
        for (KrakenSwerveModule module : modules) {
            module.setSteerCruiseVelocity(velocity);
        }
    }

    private void buildAuton() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getRobotPosition,
            this::resetPose,
            this::getRobotRelativeChassisSpeeds,
            (speeds, feedforwards) -> setRobotRelativeDrivePowers(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(1.38, 0, 0.0),
                new PIDConstants(3.3, 0.0, 0.0)),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
    }
}
