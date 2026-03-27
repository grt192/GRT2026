package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.HopperConstants;
import frc.robot.util.LoggedTalon;

public class HopperIOTalonFX implements HopperIO {
    private final LoggedTalon motor;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final MotionMagicVelocityTorqueCurrentFOC velocityControl = new MotionMagicVelocityTorqueCurrentFOC(0);

    public HopperIOTalonFX(CANBus canBus) {
        motor = new LoggedTalon(HopperConstants.KRAKEN_CAN_ID, canBus);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(HopperConstants.HOPPERINVERTED));
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(HopperConstants.STATOR_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(HopperConstants.STATOR_CURRENT_LIMIT_AMPS)));
        config.MotionMagic.MotionMagicCruiseVelocity = 300;
        config.MotionMagic.MotionMagicAcceleration = 100;
        config.MotionMagic.MotionMagicJerk = 1000;
        config.Feedback.SensorToMechanismRatio = HopperConstants.GEAR_REDUCTION;
        config.withSlot0(new Slot0Configs()
            .withKP(HopperConstants.KP)
            .withKI(HopperConstants.KI)
            .withKD(HopperConstants.KD)
            .withKS(HopperConstants.KS)
            .withKV(HopperConstants.KV));
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.positionRotations = motor.getPosition().getValueAsDouble();
        inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
        inputs.connected = motor.isConnected();
    }

    @Override
    public void setDutyCycle(double percentOutput) {
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        motor.setControl(dutyCycleControl.withOutput(percentOutput));
    }

    @Override
    public void setVelocity(double rps) {
        motor.setControl(velocityControl.withVelocity(rps));
    }

    @Override
    public void stop() {
        motor.setControl(dutyCycleControl.withOutput(0));
    }
}
