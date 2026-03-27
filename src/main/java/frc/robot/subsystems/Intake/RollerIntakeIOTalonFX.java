package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTalon;

public class RollerIntakeIOTalonFX implements RollerIntakeIO {
    private final LoggedTalon motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public RollerIntakeIOTalonFX(CANBus canBus) {
        motor = new LoggedTalon(IntakeConstants.ROLLER_CAN_ID, canBus);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(IntakeConstants.ROLLER_INVERTED));
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT)));
        config.withSlot0(new Slot0Configs()
            .withKP(IntakeConstants.ROLLER_P)
            .withKI(IntakeConstants.ROLLER_I)
            .withKD(IntakeConstants.ROLLER_D)
            .withKV(IntakeConstants.ROLLER_V));
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(RollerIntakeIOInputs inputs) {
        inputs.positionRotations = motor.getPosition().getValueAsDouble();
        inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
        inputs.dutyCycle = motor.get();
        inputs.connected = motor.isConnected();
    }

    @Override
    public void setVelocity(double velocity) {
        motor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        motor.setControl(dutyCycleRequest.withOutput(0));
    }
}
