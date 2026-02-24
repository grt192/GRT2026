package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.littletonrobotics.junction.Logger;

public class flywheel extends SubsystemBase {

    private final TalonFX upperMotor;
    private MotionMagicVelocityVoltage spinner = new MotionMagicVelocityVoltage(0);
    private DutyCycleOut dutyCycl = new DutyCycleOut(0);

    private static final String LOG_PREFIX = "FlyWheel/";

    public flywheel(CANBus cn) {
        upperMotor = new TalonFX(railgunConstants.upperId, cn);
        config();
    }

    public void config(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        /*CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);;
        cfg.withCurrentLimits(currLim);
        */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 120;   // target RPS cap
        cfg.MotionMagic.MotionMagicAcceleration = 10;    // RPS per second
        cfg.MotionMagic.MotionMagicJerk = 150;            // optional, smoothness

        cfg.Slot0.kP = 0.05;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kS = 0.2;
        cfg.Slot0.kV = 0.14;
        cfg.Slot0.kA = 0.0;

        cfg.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioUpper;

        upperMotor.getConfigurator().apply(cfg);
    }

    public void shoot(double rps){
        upperMotor.setControl(spinner.withVelocity(rps));
    }

    public double getRPS(){
        return upperMotor.getVelocity().getValueAsDouble();
    }

    public void dontShoot(){
        upperMotor.setControl(spinner.withVelocity(0));
    }

    double commandedDutyCycle = 0;
    public void flySpeed(double speed){
        commandedDutyCycle = 0;
        upperMotor.setControl(dutyCycl.withOutput(speed));
    }

    @Override
    public void periodic(){
        sendData();
    }

    public void sendData(){
        Logger.recordOutput(LOG_PREFIX + "PositionRotations",
            upperMotor.getPosition().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "VelocityRPS",
            upperMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "AppliedVolts",
            upperMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyVoltage",
            upperMotor.getSupplyVoltage().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps",
            upperMotor.getStatorCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps",
            upperMotor.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "TemperatureC",
            upperMotor.getDeviceTemp().getValueAsDouble());

        Logger.recordOutput(LOG_PREFIX + "CommandedDutyCycle",
            commandedDutyCycle);

        Logger.recordOutput(LOG_PREFIX + "Connected",
            upperMotor.isConnected());
    }

}
