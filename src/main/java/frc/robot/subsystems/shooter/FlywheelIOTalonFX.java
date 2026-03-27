package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTalon;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final LoggedTalon upperMotor;
    private final LoggedTalon secondMotor;
    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

    public FlywheelIOTalonFX(CANBus canBus) {
        upperMotor = new LoggedTalon(ShooterConstants.Flywheel.UPPER_MOTOR_ID, canBus);
        secondMotor = new LoggedTalon(ShooterConstants.Flywheel.SECOND_MOTOR_ID, canBus);
        configure();
    }

    private void configure() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = ShooterConstants.Flywheel.F_INVERTED_VALUE;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.Flywheel.MM_CRUISE_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.Flywheel.MM_ACCEL;
        cfg.MotionMagic.MotionMagicJerk = ShooterConstants.Flywheel.MM_JERK;

        Slot0Configs pidSlots = new Slot0Configs();
        pidSlots.withKP(ShooterConstants.Flywheel.KP);
        pidSlots.withKI(ShooterConstants.Flywheel.KI);
        pidSlots.withKD(ShooterConstants.Flywheel.KD);
        pidSlots.withKS(ShooterConstants.Flywheel.KS);
        pidSlots.withKV(ShooterConstants.Flywheel.KV);
        cfg.withSlot0(pidSlots);

        cfg.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEAR_RATIO;

        upperMotor.getConfigurator().apply(cfg);
        secondMotor.getConfigurator().apply(cfg);

        secondMotor.setControl(new Follower(ShooterConstants.Flywheel.UPPER_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRPS = upperMotor.getVelocity().getValueAsDouble();
        inputs.positionRotations = upperMotor.getPosition().getValueAsDouble();
        inputs.appliedVolts = upperMotor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = upperMotor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = upperMotor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = upperMotor.getDeviceTemp().getValueAsDouble();
        inputs.connected = upperMotor.isConnected();
    }

    @Override
    public void setVelocity(double rps) {
        upperMotor.setControl(velocityControl.withVelocity(rps));
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        upperMotor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        upperMotor.setControl(velocityControl.withVelocity(0));
    }
}
