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
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.Slot0Configs;

public class flywheel extends SubsystemBase {

    private final TalonFX upperMotor;
    private VelocityVoltage spinner = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private double velocity = 0;
    private final CANcoder flywheelCoder;

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
        CurrentLimitsConfigs currLim = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(50.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);;
        cfg.withCurrentLimits(currLim);
        cfg.Feedback.SensorToMechanismRatio = 1;

        cfg.Slot0.kP = 2;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kG = 1.0;

        CANcoderConfiguration ccfg = new CANcoderConfiguration();
        ccfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; 
        //ccfg.MagnetSensor.MagnetOffset = railgunConstants.upperMagnetOffset;

        flywheelCoder.getConfigurator().apply(ccfg);

        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = railgunConstants.upperEncoderId;

        cfg.Feedback.RotorToSensorRatio = railgunConstants.gearRatioUpper;

        upperMotor.getConfigurator().apply(cfg);
    }

    public void setVelocity(double vel){
        velocity = vel;
    }

    public void shoot(){
        upperMotor.setControl(spinner.withVelocity(velocity));
    }

    public void dontShoot(){
        spinner.Velocity = 0;
        upperMotor.setControl(spinner.withVelocity(0));
    }

    public void flySpeed(double speed){
        upperMotor.setControl(dutyCycl.withOutput(speed));
    }
}
