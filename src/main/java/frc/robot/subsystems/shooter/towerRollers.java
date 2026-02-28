package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TowerConstants.INTAKE;
import frc.robot.util.LoggedTalon;


public class towerRollers extends SubsystemBase {

    private final LoggedTalon krakenMotor;
    private final MotionMagicVelocityTorqueCurrentFOC velocityControl;
    private final DutyCycleOut dutyCycleControl;
    public towerRollers(CANBus canBus) {
        krakenMotor = new LoggedTalon(TowerConstants.KRAKEN_CAN_ID, canBus);
        velocityControl = new MotionMagicVelocityTorqueCurrentFOC(0);
        dutyCycleControl = new DutyCycleOut(0);

        configureMotor();
    }


    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor output
        config.withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(TowerConstants.HOPPERINVERTED));

        // Current limits
        config.withCurrentLimits(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(TowerConstants.STATOR_CURRENT_LIMIT_ENABLE)
                        .withStatorCurrentLimit(Amps.of(TowerConstants.STATOR_CURRENT_LIMIT_AMPS))
        );

        config.MotionMagic.MotionMagicAcceleration = 100;    // rotations/sec^2
        config.MotionMagic.MotionMagicJerk = 1000;           // optional

        // Velocity control PID (Slot 0)
        config.withSlot0(new Slot0Configs()
                .withKP(TowerConstants.KP)
                .withKI(TowerConstants.KI)
                .withKD(TowerConstants.KD)
                .withKS(TowerConstants.KS)
                .withKV(TowerConstants.KV)
        );

        krakenMotor.getConfigurator().apply(config);
    }
    public void setTower(INTAKE state){
        switch (state) {
            case BALLUP:
                krakenMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(TowerConstants.TARGET_RPS));
                break;
            case BALLDOWN:
                krakenMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(-TowerConstants.TARGET_RPS));
                break;
            case STOP:
                krakenMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(0.0));
                break;
        }
    }
    public void setManualControl(double percentOutput) {
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));
        krakenMotor.setControl(dutyCycleControl.withOutput(percentOutput));
    }

    @Override
    public void periodic() {
        krakenMotor.updateDashboard();
    }
}