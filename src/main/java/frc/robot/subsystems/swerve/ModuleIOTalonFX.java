package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.Constants.SwerveDriveConstants.*;
import static frc.robot.Constants.SwerveSteerConstants.*;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private final VelocityTorqueCurrentFOC driveTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final PositionTorqueCurrentFOC steerPositionControl = new PositionTorqueCurrentFOC(0)
        .withSlot(0).withUpdateFreqHz(100.0);

    private double targetDriveRPS = 0;

    private StatusSignal<Angle> drivePositionSignal;
    private StatusSignal<AngularVelocity> driveVelocitySignal;
    private StatusSignal<Voltage> driveAppliedVoltsSignal;
    private StatusSignal<Current> driveSupplyCurrentSignal;
    private StatusSignal<Current> driveTorqueCurrentSignal;

    public ModuleIOTalonFX(int drivePort, int steerPort, int canCoderPort, CANBus canBus) {
        driveMotor = new TalonFX(drivePort, canBus);
        steerMotor = new TalonFX(steerPort, canBus);
        steerEncoder = new CANcoder(canCoderPort);

        driveMotor.setPosition(0);

        configureDriveMotor();
        configureSteerMotor();
        initSignals();
    }

    private void configureDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_PEAK_STATOR_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_PEAK_STATOR_CURRENT;
        config.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_CURRENT_LIMIT_ENABLE;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = DRIVE_CURRENT_LIMIT_ENABLE;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DRIVE_RAMP_RATE;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicCruiseVelocity = DRIVE_MAX_VELOCITY_RPS;
        config.MotionMagic.MotionMagicAcceleration = DRIVE_MAX_ACCELERATION;

        for (int i = 0; i < 5; i++) {
            if (driveMotor.getConfigurator().apply(config, 0.1) == StatusCode.OK)
                break;
        }
    }

    private void configureSteerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.TorqueCurrent.PeakForwardTorqueCurrent = STEER_PEAK_STATOR_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -STEER_PEAK_STATOR_CURRENT;
        config.CurrentLimits.SupplyCurrentLimit = STEER_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = STEER_CURRENT_LIMIT_ENABLE;
        config.CurrentLimits.StatorCurrentLimit = STEER_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = STEER_CURRENT_LIMIT_ENABLE;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = STEER_RAMP_RATE;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.MotionMagic.MotionMagicCruiseVelocity = STEER_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = STEER_ACCELERATION;

        for (int i = 0; i < 5; i++) {
            if (steerMotor.getConfigurator().apply(config, 0.1) == StatusCode.OK)
                break;
        }
        steerMotor.setPosition(0);
    }

    private void initSignals() {
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveAppliedVoltsSignal = driveMotor.getMotorVoltage();
        driveTorqueCurrentSignal = driveMotor.getTorqueCurrent();
        driveSupplyCurrentSignal = driveMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0, drivePositionSignal, driveVelocitySignal,
            driveAppliedVoltsSignal, driveTorqueCurrentSignal, driveSupplyCurrentSignal);
        driveMotor.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(drivePositionSignal, driveVelocitySignal,
            driveAppliedVoltsSignal, driveTorqueCurrentSignal, driveSupplyCurrentSignal);

        inputs.drivePositionMeters = DRIVE_WHEEL_CIRCUMFERENCE / DRIVE_GEAR_REDUCTION
            * drivePositionSignal.getValueAsDouble();
        inputs.driveVelocityMPS = driveVelocitySignal.getValueAsDouble()
            / DRIVE_GEAR_REDUCTION * DRIVE_WHEEL_CIRCUMFERENCE;
        inputs.driveAppliedVolts = driveAppliedVoltsSignal.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrentSignal.getValueAsDouble();
        inputs.driveTorqueCurrentAmps = driveTorqueCurrentSignal.getValueAsDouble();
        inputs.driveTemperatureCelsius = driveMotor.getDeviceTemp().getValueAsDouble();
        inputs.driveTargetRPS = targetDriveRPS;

        inputs.steerPositionRotations = steerMotor.getPosition().getValueAsDouble();
        inputs.steerVelocityRPM = steerMotor.getVelocity().getValueAsDouble() * STEER_GEAR_REDUCTION * 60.0;
        inputs.steerAppliedVolts = steerMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerSupplyCurrentAmps = steerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.steerTorqueCurrentAmps = steerMotor.getTorqueCurrent().getValueAsDouble();
        inputs.steerTemperatureCelsius = steerMotor.getDeviceTemp().getValueAsDouble();
        inputs.steerClosedLoopError = steerMotor.getClosedLoopError().getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(double metersPerSec) {
        targetDriveRPS = metersPerSec / DRIVE_WHEEL_CIRCUMFERENCE * DRIVE_GEAR_REDUCTION;
        driveMotor.setControl(driveTorqueCurrentFOC.withVelocity(targetDriveRPS));
    }

    @Override
    public void setSteerPosition(double radians) {
        double rotations = (radians / (2 * Math.PI)) + 0.5;
        steerMotor.setControl(steerPositionControl.withPosition(rotations));
    }

    @Override
    public void setSteerCruiseVelocity(double velocity) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicCruiseVelocity = velocity;
        mmConfigs.MotionMagicAcceleration = STEER_ACCELERATION;
        steerMotor.getConfigurator().apply(mmConfigs);
    }

    @Override
    public void configureDrivePID(double p, double i, double d, double s, double v) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = p;
        slot0.kI = i;
        slot0.kD = d;
        slot0.kS = s;
        slot0.kV = v;
        driveMotor.getConfigurator().apply(slot0);
    }

    @Override
    public void configureSteerPID(double p, double i, double d, double s) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = p;
        slot0.kI = i;
        slot0.kD = d;
        slot0.kS = s;
        steerMotor.getConfigurator().apply(slot0);
    }
}
