package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;
import frc.robot.util.LoggedTalon;

public class WinchSubsystem extends SubsystemBase {
    private LoggedTalon motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private TorqueCurrentFOC torqueCurrentControl = new TorqueCurrentFOC(0);
    private CoastOut coast = new CoastOut();

    private final StatusSignal<Boolean> forwardLimitSignal;
    private final StatusSignal<Boolean> reverseLimitSignal;

    private CANrange canRange;
    private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
    private Trigger homeTrigger;
    private Trigger deployedTrigger;

    public WinchSubsystem(CANBus canBusObj) {
        motor = new LoggedTalon(ClimbConstants.WINCH_MOTOR_CAN_ID, canBusObj, "Winch");
        canRange = new CANrange(ClimbConstants.CANRANGE_CAN_ID, canBusObj);

        configureCANrange();
        configureMotor();

        forwardLimitSignal = motor.getFault_ForwardSoftLimit();
        reverseLimitSignal = motor.getFault_ReverseSoftLimit();
        BaseStatusSignal.setUpdateFrequencyForAll(50, forwardLimitSignal, reverseLimitSignal);

        homeTrigger = new Trigger(() -> isAtDistance(ClimbConstants.WINCH_HOME_DISTANCE));
        deployedTrigger = new Trigger(() -> isAtDistance(ClimbConstants.WINCH_DEPLOYED_DISTANCE));

        homeTrigger.onTrue(this.runOnce(() -> motor.setControl(coast)));
        deployedTrigger.onTrue(this.runOnce(() -> motor.setControl(coast)));
    }

    private void configureMotor() {
        motorConfig.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120)))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(ClimbConstants.WINCH_MOTOR_INVERTED))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(ClimbConstants.WINCH_GR));

        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                System.out.println("MOTOR " + motor.getDeviceID() + " CONFIGURED!");
                break;
            }
            if (i == 4) {
                System.out.println("VERY BAD, MOTOR " + motor.getDeviceID() + " DID NOT GET CONFIGURED");
            }
        }
    }

    private void configureCANrange() {
        for (int i = 0; i < 5; i++) {
            if (canRange.getConfigurator().apply(canRangeConfig, 0.1) == StatusCode.OK) {
                System.out.println("CANRANGE " + canRange.getDeviceID() + " CONFIGURED!");
                break;
            }
            if (i == 4) {
                System.out.println("VERY BAD, CANRANGE " + canRange.getDeviceID() + " DID NOT GET CONFIGURED");
            }
        }
    }

    private void logToDashboard() {
        Logger.recordOutput(ClimbConstants.WINCH_TABLE + "/state", getWinchState().toString());

        Logger.recordOutput(ClimbConstants.WINCH_TABLE + "/distance(mm)", getDistance().in(Millimeters));

        Logger.recordOutput(ClimbConstants.WINCH_TABLE + "/forwardSoftStop", isAtHome());
        Logger.recordOutput(ClimbConstants.WINCH_TABLE + "/reverseSoftStop", isAtDeployed());
    }

    public void setMotorDutyCycle(double dutyCycle) {
        dutyCycle = (isAtHome() && dutyCycle > 0) ? 0 : dutyCycle;
        dutyCycle = (isAtDeployed() && dutyCycle < 0) ? 0 : dutyCycle;

        dutyCycle = Math.max(-1.0, Math.min(dutyCycle, 1.0));
        dutyCycle *= ClimbConstants.WINCH_MAX_OUTPUT;
        dutyCycleControl.withOutput(dutyCycle);
        motor.setControl(dutyCycleControl);
    }

    public double getDutyCycleSetpoint() {
        return dutyCycleControl.Output;
    }

    public String getControlMode() {
        return motor.getControlMode().toString();
    }

    public void stop() {
        dutyCycleControl.withOutput(0);
        motor.setControl(dutyCycleControl);
    }

    public void setTorqueCurrent(double amps) {
        amps = (isAtHome() && amps > 0) ? 0 : amps;
        amps = (isAtDeployed() && amps < 0) ? 0 : amps;

        torqueCurrentControl.withOutput(amps);
        motor.setControl(torqueCurrentControl);
    }

    public Distance getDistance() {
        return canRange.getDistance(false).getValue();
    }

    public boolean isAtDistance(Distance target) {
        double errorMili = Math.abs(getDistance().in(Millimeters) - target.in(Millimeters));
        return errorMili <= ClimbConstants.WINCH_DISTANCE_TOLERANCE.in(Millimeters);
    }

    public Current getSupplyCurrent() {
        return motor.getSupplyCurrent(false).getValue();
    }

    public Current getTorqueCurrent() {
        return motor.getTorqueCurrent(false).getValue();
    }

    public boolean isAtHome() {
        return homeTrigger.getAsBoolean();
    }

    public boolean isAtDeployed() {
        return deployedTrigger.getAsBoolean();
    }

    public CLIMB_MECH_STATE getWinchState() {
        if (isAtDistance(ClimbConstants.WINCH_HOME_DISTANCE)) {
            return CLIMB_MECH_STATE.HOME;
        } else if (isAtDistance(ClimbConstants.WINCH_DEPLOYED_DISTANCE)) {
            return CLIMB_MECH_STATE.DEPLOYED;
        } else {
            return CLIMB_MECH_STATE.FLOATING;
        }
    }

    @Override
    public void periodic() {
        motor.updateDashboard();
        logToDashboard();
    }
}
