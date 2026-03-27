package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTalon;

public class HoodIOTalonFX implements HoodIO {
    private final LoggedTalon motor;
    private final CANcoder encoder;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);

    public HoodIOTalonFX(CANBus canBus) {
        motor = new LoggedTalon(ShooterConstants.Hood.MOTOR_ID, canBus);
        encoder = new CANcoder(ShooterConstants.Hood.ENCODER_ID, canBus);
        configure();
    }

    private void configure() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ShooterConstants.Hood.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(ShooterConstants.Hood.CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(ShooterConstants.Hood.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(ShooterConstants.Hood.CURRENT_LIMIT_ENABLE));
        cfg.Feedback.RotorToSensorRatio = ShooterConstants.Hood.GEAR_RATIO;
        cfg.Slot0.kP = ShooterConstants.Hood.KP;
        cfg.Slot0.kI = ShooterConstants.Hood.KI;

        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        ccfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        ccfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoder.getConfigurator().apply(ccfg);

        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = ShooterConstants.Hood.ENCODER_ID;
        cfg.Feedback.SensorToMechanismRatio = 1.0;

        motor.getConfigurator().apply(cfg);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRotations = motor.getPosition().getValueAsDouble();
        inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
        inputs.connected = motor.isConnected();
    }

    @Override
    public void setPosition(double rotations) {
        motor.setControl(positionControl.withPosition(rotations));
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        motor.setControl(dutyCycleControl.withOutput(0));
    }
}
