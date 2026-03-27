package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.TowerConstants;
import frc.robot.util.LoggedTalon;

public class TowerRollersIOTalonFX implements TowerRollersIO {
    private final LoggedTalon motor;
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final MotionMagicVelocityTorqueCurrentFOC velocityControl = new MotionMagicVelocityTorqueCurrentFOC(0);

    public TowerRollersIOTalonFX(CANBus canBus) {
        motor = new LoggedTalon(TowerConstants.KRAKEN_CAN_ID, canBus);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(TowerConstants.HOPPERINVERTED));
        config.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(TowerConstants.STATOR_CURRENT_LIMIT_ENABLE)
                .withStatorCurrentLimit(Amps.of(TowerConstants.STATOR_CURRENT_LIMIT_AMPS)));
        config.MotionMagic.MotionMagicCruiseVelocity = TowerConstants.MM_MAXVELO;
        config.MotionMagic.MotionMagicAcceleration = TowerConstants.MM_ACCEL;
        config.MotionMagic.MotionMagicJerk = TowerConstants.MM_JERK;
        config.Feedback.SensorToMechanismRatio = TowerConstants.GEAR_REDUCTION;
        Slot0Configs pidSlots = new Slot0Configs();
        pidSlots.withKP(TowerConstants.KP);
        pidSlots.withKI(TowerConstants.KI);
        pidSlots.withKD(TowerConstants.KD);
        pidSlots.withKS(TowerConstants.KS);
        pidSlots.withKV(TowerConstants.KV);
        config.withSlot0(pidSlots);
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(TowerRollersIOInputs inputs) {
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
