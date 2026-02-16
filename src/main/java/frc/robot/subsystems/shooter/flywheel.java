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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import org.littletonrobotics.junction.Logger;

public class flywheel extends SubsystemBase {

    private final TalonFX upperMotor;
    private MotionMagicVelocityVoltage spinner;
    private DutyCycleOut dutyCycl;
    private double velocity = 0;
    private final CANcoder flywheelCoder;

    private static final String LOG_PREFIX = "FlyWheel/";

    public flywheel(CANBus cn) {
        // Construct motors directly on the CAN bus
        upperMotor = new TalonFX(railgunConstants.upperId, cn);
        flywheelCoder = new CANcoder(railgunConstants.upperEncoderId, cn);
        config();
       
    }

    public void config(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        /*CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);;
        cfg.withCurrentLimits(currLim);
        */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 120;   // target RPS cap
        cfg.MotionMagic.MotionMagicAcceleration = 160;    // RPS per second
        cfg.MotionMagic.MotionMagicJerk = 800;            // optional, smoothness

        cfg.Slot0.kP = 2;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kV = 0.12;
        cfg.Slot0.kA = 0.0;

        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        ccfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; 
        //flywheelCoder.getConfigurator().apply(ccfg);

        //cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        //cfg.Feedback.FeedbackRemoteSensorID = railgunConstants.upperEncoderId;

        cfg.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioUpper;

        upperMotor.getConfigurator().apply(cfg);
    }

    public void setVelocity(double vel){
        velocity = vel;
    }

    public void shoot(){
        upperMotor.setControl(spinner.withVelocity(velocity));
    }

    public void dontShoot(){
        upperMotor.setControl(spinner.withVelocity(0));
    }

    public void flySpeed(double speed){
        velocity = speed;
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
            velocity);

        Logger.recordOutput(LOG_PREFIX + "Connected",
            upperMotor.isConnected());
    }

}
