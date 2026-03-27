package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTalon;

public class PivotIntakeIOTalonFX implements PivotIntakeIO {
    private final LoggedTalon motor;
    private final CANcoder canCoder;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

    public PivotIntakeIOTalonFX(CANBus canBus) {
        motor = new LoggedTalon(IntakeConstants.PIVOT_MOTOR_ID, canBus);
        canCoder = new CANcoder(IntakeConstants.PIVOT_CANCODER_ID, canBus);
        configEncoder();
        configMotor();
    }

    private void configEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = 0.0;
        canCoder.getConfigurator().apply(config);
    }

    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = IntakeConstants.PIVOT_P;
        config.Slot0.kI = IntakeConstants.PIVOT_I;
        config.Slot0.kD = IntakeConstants.PIVOT_D;
        config.Slot0.kV = IntakeConstants.PIVOT_F;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(IntakeConstants.PIVOT_CANCODER_ID)
            .withSensorToMechanismRatio(1.0)
            .withRotorToSensorRatio(IntakeConstants.GEAR_RATIO));
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT_ENABLE));
        config.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(IntakeConstants.PIVOT_CRUISE_VELOCITY)
                .withMotionMagicAcceleration(IntakeConstants.PIVOT_ACCELERATION));
        config.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(IntakeConstants.TOP_LIMIT)
                .withReverseSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(IntakeConstants.BOTTOM_LIMIT));
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(PivotIntakeIOInputs inputs) {
        inputs.motorPositionRotations = motor.getPosition().getValueAsDouble();
        inputs.motorVelocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
        inputs.closedLoopError = motor.getClosedLoopError().getValueAsDouble();
        inputs.motorConnected = motor.isConnected();
        inputs.encoderPositionRotations = canCoder.getPosition().getValueAsDouble();
        inputs.encoderAbsolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
        inputs.encoderConnected = canCoder.getPosition().getStatus().isOK();
    }

    @Override
    public void setPosition(double rotations) {
        motor.setControl(motionMagicControl.withPosition(rotations));
    }

    @Override
    public void setManualSpeed(double speed) {
        motor.setControl(dutyCycleControl.withOutput(speed));
    }

    @Override
    public void stop() {
        motor.setControl(dutyCycleControl.withOutput(0));
    }

    @Override
    public void zeroEncoder() {
        canCoder.setPosition(0.0);
        motor.setPosition(0.0);
    }

    @Override
    public void setEncoderToMax() {
        double maxRotations = IntakeConstants.PIVOT_OUT_POS;
        canCoder.setPosition(maxRotations);
        motor.setPosition(maxRotations);
    }
}
