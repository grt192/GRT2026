package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class hood extends SubsystemBase {

    private final TalonFX hoodMotor;
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private CANdi limit;
    private PositionTorqueCurrentFOC focThing = new PositionTorqueCurrentFOC(0);
    private final CANcoder hoodCoder;

    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    private double commandedDutyCycle = 0.0;
    private static final String LOG_PREFIX = "Hood/";

    public hood(CANBus cn) {
        hoodMotor = new TalonFX(railgunConstants.hoodId, cn);
        limit = new CANdi(railgunConstants.limitId, cn);
        hoodCoder = new CANcoder(railgunConstants.hoodEncoderId, cn);

        config();

        double hoodRot = hoodCoder.getAbsolutePosition().refresh().getValueAsDouble();
        hoodMotor.setPosition(hoodRot);

        // Initialize status signals
        velocitySignal = hoodMotor.getVelocity();
        positionSignal = hoodMotor.getPosition();
        supplyVoltageSignal = hoodMotor.getSupplyVoltage();
        motorVoltageSignal = hoodMotor.getMotorVoltage();
        statorCurrentSignal = hoodMotor.getStatorCurrent();
        supplyCurrentSignal = hoodMotor.getSupplyCurrent();
        temperatureSignal = hoodMotor.getDeviceTemp();

        // Set update frequency for all signals (optimize CAN bus usage)
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, // 50 Hz update rate
            velocitySignal,
            positionSignal,
            supplyVoltageSignal,
            motorVoltageSignal,
            statorCurrentSignal,
            supplyCurrentSignal,
            temperatureSignal
        );

        // Optimize bus utilization
        hoodMotor.optimizeBusUtilization();
    }

    public void config(){
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);;
        cfg.withCurrentLimits(currLim);
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = railgunConstants.upperAngle;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = railgunConstants.lowerAngle;
        cfg.Feedback.SensorToMechanismRatio = 1;

        cfg.Slot0.kP = 2;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kG = 1.0;

        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        ccfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; 
        ccfg.MagnetSensor.MagnetOffset = railgunConstants.hoodMagnetOffset;

        hoodCoder.getConfigurator().apply(ccfg);

        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = railgunConstants.hoodEncoderId;

        cfg.Feedback.RotorToSensorRatio = railgunConstants.gearRatioHood;

        hoodMotor.getConfigurator().apply(cfg);
    }

    public void setHoodAngle(double rotationAngle){
        if(rotationAngle >= railgunConstants.lowerAngle && rotationAngle <= railgunConstants.upperAngle){
            hoodMotor.setControl(focThing.withPosition(rotationAngle));
        }
    }

    public void hoodSpeed(double speed){
        commandedDutyCycle = speed;
        
        double pos = hoodMotor.getPosition().refresh().getValueAsDouble();
        if(pos >= railgunConstants.upperAngle && speed >0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
        }else if(pos <= railgunConstants.lowerAngle && speed <0){
            hoodMotor.setControl(dutyCycl.withOutput(0));
        }else{
            hoodMotor.setControl(dutyCycl.withOutput(speed));
        }
        
    }

    boolean prevPress = false;
    @Override
    public void periodic(){
        if(limit.getS1Closed().refresh().getValue() && !prevPress){
            hoodMotor.setPosition(railgunConstants.initHoodAngle);
            prevPress = true;
        }
        
        if(!limit.getS1Closed().refresh().getValue()){
            prevPress = false;
        }

        sendData();
            
    }

    public void sendData(){
        BaseStatusSignal.refreshAll(
            velocitySignal,
            positionSignal,
            supplyVoltageSignal,
            motorVoltageSignal,
            statorCurrentSignal,
            supplyCurrentSignal,
            temperatureSignal
        );

        // Motor state logging
        Logger.recordOutput(LOG_PREFIX + "VelocityRPS", velocitySignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "VelocityRPM", velocitySignal.getValueAsDouble() * 60.0);
        Logger.recordOutput(LOG_PREFIX + "PositionRotations", positionSignal.getValueAsDouble());

        // Electrical logging
        Logger.recordOutput(LOG_PREFIX + "SupplyVoltage", supplyVoltageSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "MotorVoltage", motorVoltageSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "StatorCurrentAmps", statorCurrentSignal.getValueAsDouble());
        Logger.recordOutput(LOG_PREFIX + "SupplyCurrentAmps", supplyCurrentSignal.getValueAsDouble());

        // Thermal logging
        Logger.recordOutput(LOG_PREFIX + "TemperatureCelsius", temperatureSignal.getValueAsDouble());

        // Command logging
        Logger.recordOutput(LOG_PREFIX + "CommandedDutyCycle", commandedDutyCycle);
        Logger.recordOutput(LOG_PREFIX + "CommandedVoltage", commandedDutyCycle * supplyVoltageSignal.getValueAsDouble());

        // Calculated metrics
        double power = motorVoltageSignal.getValueAsDouble() * statorCurrentSignal.getValueAsDouble();
        Logger.recordOutput(LOG_PREFIX + "PowerWatts", power);

        // Connection status
        Logger.recordOutput(LOG_PREFIX + "Connected", hoodMotor.isConnected());
        Logger.recordOutput("Torque", hoodMotor.getTorqueCurrent().getValueAsDouble() * hoodMotor.getMotorKT().getValueAsDouble());
    }

}
