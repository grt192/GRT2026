package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;
import frc.robot.util.LoggedTalon;

public class StabilizingArmSubsystem extends SubsystemBase {

    private LoggedTalon motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private PositionTorqueCurrentFOC posControl = new PositionTorqueCurrentFOC(0).withSlot(0);

    private final StatusSignal<Boolean> forwardLimitSignal;
    private final StatusSignal<Boolean> reverseLimitSignal;

    public StabilizingArmSubsystem(CANBus canBusObj) {
        motor = new LoggedTalon(ClimbConstants.ARM_MOTOR_CAN_ID, canBusObj, "Arm");
        configureMotor();

        forwardLimitSignal = motor.getFault_ForwardSoftLimit();
        reverseLimitSignal = motor.getFault_ReverseSoftLimit();
        // Change soft limit signal update frequency
        // idk why this is necessary but it makes code work
        BaseStatusSignal.setUpdateFrequencyForAll(50, forwardLimitSignal, reverseLimitSignal);

        homeEncoder();
    }

    private void configureMotor() {
        motorConfig.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120)))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(ClimbConstants.ARM_MOTOR_INVERTED))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(ClimbConstants.ARM_GR))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ClimbConstants.ARM_FORWARD_LIMIT)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ClimbConstants.ARM_REVERSE_LIMIT))
            .withSlot0(new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(ClimbConstants.ARM_kP)
                .withKI(ClimbConstants.ARM_kI)
                .withKD(ClimbConstants.ARM_kD)
                .withKG(ClimbConstants.ARM_kG)
                .withKS(ClimbConstants.ARM_kS));

        for (int i = 0; i < 5; i++) {
            if (motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK) {
                System.out.println("MOTOR " + motor.getDeviceID() + " CONFIGURED!");
                break; // Success
            }
            if (i == 4) {
                System.out.println("VERY BAD, MOTOR " + motor.getDeviceID() + " DID NOT GET CONFIGURED");
            }
        }
    }

    // take an input value and clamp it to the max value then run motor at that duty
    // cycle
    public void setMotorDutyCycle(double dutyCycle) {
        dutyCycle = Math.max(-1.0, Math.min(dutyCycle, 1.0));
        dutyCycle *= ClimbConstants.ARM_MAX_OUTPUT;
        dutyCycleControl.withOutput(dutyCycle);
        motor.setControl(dutyCycleControl);
    }

    public void stop() {
        dutyCycleControl.withOutput(0);
        motor.setControl(dutyCycleControl);
    }

    public double getDutyCycleSetpoint() {
        return dutyCycleControl.Output;
    }

    public void setPositionSetpoint(Angle setpoint) {
        if (setpoint.gt(ClimbConstants.ARM_FORWARD_LIMIT)) {
            setpoint = ClimbConstants.ARM_FORWARD_LIMIT;
        } else if (setpoint.lt(ClimbConstants.ARM_REVERSE_LIMIT)) {
            setpoint = ClimbConstants.ARM_REVERSE_LIMIT;
        }

        posControl.withPosition(setpoint);
        motor.setControl(posControl);
    }

    public Optional<Angle> getPositionSetpoint() {
        if (motor.getControlMode(false).getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
            return Optional.empty();
        }
        return Optional.of(Rotations.of(motor.getClosedLoopReference(false).getValue()));
    }

    // Checks if arm is at position set by PID control with a tolerance
    public boolean atSetPosition() {
        // if not in position control return false
        if (motor.getControlMode(false).getValue() != ControlModeValue.PositionTorqueCurrentFOC) {
            return false;
        }

        // checks if closed loop error is within tolerance
        return (Rotations.of(Math.abs(motor.getClosedLoopError(false).getValue())))
            .lte(ClimbConstants.ARM_POSITION_TOLERANCE);
    }

    // Checks if arm is at given position
    public boolean atPosition(Angle target) {
        // finds error between position and target and its absolute value
        Angle error = target.minus(getMotorPosition());
        Angle absError = Radians.of(Math.abs(error.in(Radians)));

        // checks if difference is within tolerance
        return absError.lte(ClimbConstants.ARM_POSITION_TOLERANCE);
    }

    public void homeEncoder() {
        setEncoder(ClimbConstants.ARM_HOME_POS);
    }

    public void setEncoder(Angle pos) {
        motor.setPosition(pos);
    }

    public Angle getMotorPosition() {
        return motor.getPosition(false).getValue();
    }

    public Optional<Boolean> getForwardLimit() {
        boolean forwardLimit = forwardLimitSignal.refresh().getValue();
        if (!forwardLimitSignal.hasUpdated() || forwardLimitSignal.getStatus() != StatusCode.OK) {
            return Optional.empty();
        }
        return Optional.of(forwardLimit);
    }

    public Optional<Boolean> getReverseLimit() {
        boolean reverseLimit = reverseLimitSignal.refresh().getValue();
        if (!reverseLimitSignal.hasUpdated() || reverseLimitSignal.getStatus() != StatusCode.OK) {
            return Optional.empty();
        }
        return Optional.of(reverseLimit);
    }

    public boolean isForwardLimitActive() {
        return getForwardLimit().orElse(false);
    }

    public boolean isReverseLimitActive() {
        return getReverseLimit().orElse(false);
    }

    public CLIMB_MECH_STATE getArmState() {
        if (atPosition(ClimbConstants.ARM_HOME_POS)) {
            return CLIMB_MECH_STATE.HOME;
        } else if (atPosition(ClimbConstants.ARM_DEPLOYED_POS)) {
            return CLIMB_MECH_STATE.DEPLOYED;
        } else {
            return CLIMB_MECH_STATE.FLOATING;
        }
    }

    private void logToDashboard() {
        Logger.recordOutput(ClimbConstants.ARM_TABLE + "/state", getArmState().toString());


        Logger.recordOutput(ClimbConstants.ARM_TABLE + "/forwardSoftStop", isForwardLimitActive());
        Logger.recordOutput(ClimbConstants.ARM_TABLE + "/reverseSoftStop", isReverseLimitActive());
    }

    public Current getSupplyCurrent() {
        return motor.getSupplyCurrent(false).getValue();
    }

    public Current getTorqueCurrent() {
        return motor.getTorqueCurrent(false).getValue();
    }

    public String getControlMode() {
        return motor.getControlMode(false).toString();
    }

    @Override
    public void periodic() {
        motor.updateDashboard();
        logToDashboard();
    }
}
