package frc.robot.subsystems.shooter;

import frc.robot.Constants.railgunConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class flywheel extends SubsystemBase {

    private final TalonFX upperMotor;
    private VelocityVoltage spinner = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private double velocity = 0;

    public flywheel(CANBus cn) {
        // Construct motors directly on the CAN bus
        upperMotor = new TalonFX(railgunConstants.upperId, cn);
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
        cfg.Feedback.SensorToMechanismRatio = railgunConstants.gearRatioUpper;

        cfg.Slot0.kP = 2;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kG = 1.0;

        upperMotor.getConfigurator().apply(cfg);
    }

    public void setVelocity(double vel){
        velocity = vel;
    }

    public void shoot(){
        spinner.Velocity = velocity;
        upperMotor.setControl(new VelocityVoltage(spinner.Velocity));
    }

    public void dontShoot(){
        spinner.Velocity = 0;
        upperMotor.setControl(new VelocityVoltage(spinner.Velocity));
    }

    public void flySpeed(double speed){
        upperMotor.setControl(dutyCycl.withOutput(speed));
    }
}
