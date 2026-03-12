package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.RobotEnableValue;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class LoggedTalon extends TalonFX {
    enum TelemetryLevel {
        LESS,
        STANDARD,
        DEBUG,
        MORE,
        DTM;

        boolean includes(TelemetryLevel required) {
            return this.ordinal() >= required.ordinal();
        }
    }

    /** A signal paired with its telemetry level and logging action. */
    static class LoggableSignal {
        final BaseStatusSignal signal;
        final TelemetryLevel level;
        final Runnable logger;

        LoggableSignal(BaseStatusSignal signal, TelemetryLevel level, Runnable logger) {
            this.signal = signal;
            this.level = level;
            this.logger = logger;
        }
    }

    private static final TelemetryLevel DEFAULT_TELEMETRY_LEVEL = TelemetryLevel.STANDARD;
    private String logPrefix;

    public TelemetryLevel telemetryLevel = DEFAULT_TELEMETRY_LEVEL;

    /** All LoggableSignals, populated during field initialization via {@link #logged}. */
    private final List<LoggableSignal> allSignals = new ArrayList<>();

    /** Curated set of signals to refresh and log, filtered by telemetry level. */
    private final Set<LoggableSignal> signalsToLog = new LinkedHashSet<>();

    /** Creates a LoggableSignal, registers it in {@link #allSignals}, and returns it. */
    private LoggableSignal logged(BaseStatusSignal signal, TelemetryLevel level, Runnable logger) {
        LoggableSignal ls = new LoggableSignal(signal, level, logger);
        allSignals.add(ls);
        return ls;
    }

    /** Add a LoggableSignal to the active logging set. */
    public void addToLogging(LoggableSignal ls) {
        signalsToLog.add(ls);
    }

    /** Remove a LoggableSignal from the active logging set. */
    public void removeFromLogging(LoggableSignal ls) {
        signalsToLog.remove(ls);
    }

    // ===== Cached signals (refresh once, then read many) =====

    // --- Position / Velocity / Acceleration ---
    private final StatusSignal<Angle> position = getPosition(false); // STANDARD
    public final LoggableSignal loggedPosition = logged(position, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/Position", position.getValue().in(Units.Radians)));

    private final StatusSignal<AngularVelocity> velocity = getVelocity(false); // STANDARD
    public final LoggableSignal loggedVelocity = logged(velocity, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/Velocity", velocity.getValue().in(Units.RadiansPerSecond)));

    private final StatusSignal<AngularAcceleration> acceleration = getAcceleration(false); // MORE
    public final LoggableSignal loggedAcceleration = logged(acceleration, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/Acceleration",
            acceleration.getValue().in(Units.RadiansPerSecondPerSecond)));

    private final StatusSignal<Angle> rotorPosition = getRotorPosition(false); // MORE
    public final LoggableSignal loggedRotorPosition = logged(rotorPosition, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/RotorPosition", rotorPosition.getValue().in(Units.Radians)));

    private final StatusSignal<AngularVelocity> rotorVelocity = getRotorVelocity(false); // MORE
    public final LoggableSignal loggedRotorVelocity = logged(rotorVelocity, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/RotorVelocity",
            rotorVelocity.getValue().in(Units.RadiansPerSecond)));

    // --- Duty cycle / Voltage ---
    private final StatusSignal<Double> dutyCycle = getDutyCycle(false); // STANDARD
    public final LoggableSignal loggedDutyCycle = logged(dutyCycle, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/DutyCycle", dutyCycle.getValue()));

    private final StatusSignal<Voltage> supplyVoltage = getSupplyVoltage(false); // STANDARD
    public final LoggableSignal loggedSupplyVoltage = logged(supplyVoltage, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/SupplyVoltage", supplyVoltage.getValue().in(Units.Volts)));

    private final StatusSignal<Voltage> appliedVoltage = getMotorVoltage(false); // STANDARD
    public final LoggableSignal loggedAppliedVoltage = logged(appliedVoltage, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/AppliedVoltage", appliedVoltage.getValue().in(Units.Volts)));

    // --- Current ---
    private final StatusSignal<Current> supplyCurrent = getSupplyCurrent(false); // LESS
    public final LoggableSignal loggedSupplyCurrent = logged(supplyCurrent, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/SupplyCurrent", supplyCurrent.getValue().in(Units.Amps)));

    private final StatusSignal<Current> statorCurrent = getStatorCurrent(false); // STANDARD
    public final LoggableSignal loggedStatorCurrent = logged(statorCurrent, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/StatorCurrent", statorCurrent.getValue().in(Units.Amps)));

    private final StatusSignal<Current> torqueCurrent = getTorqueCurrent(false); // STANDARD
    public final LoggableSignal loggedTorqueCurrent = logged(torqueCurrent, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/TorqueCurrent", torqueCurrent.getValue().in(Units.Amps)));

    // --- Temperature ---
    private final StatusSignal<Temperature> deviceTemp = getDeviceTemp(false); // LESS
    public final LoggableSignal loggedDeviceTemp = logged(deviceTemp, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/TempC", deviceTemp.getValue().in(Units.Celsius)));

    private final StatusSignal<Temperature> processorTemp = getProcessorTemp(false); // MORE
    public final LoggableSignal loggedProcessorTemp = logged(processorTemp, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/ProcessorTempC", processorTemp.getValue().in(Units.Celsius)));

    // --- Control mode / output ---
    private final StatusSignal<ControlModeValue> controlMode = getControlMode(false); // STANDARD
    public final LoggableSignal loggedControlMode = logged(controlMode, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/ControlMode", (double) controlMode.getValue().ordinal()));

    private final StatusSignal<AppliedRotorPolarityValue> appliedRotorPolarity = getAppliedRotorPolarity(false); // DEBUG
    public final LoggableSignal loggedAppliedRotorPolarity = logged(appliedRotorPolarity, TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/AppliedRotorPolarity",
            appliedRotorPolarity.getValue().name()));

    private final StatusSignal<BridgeOutputValue> bridgeOutput = getBridgeOutput(false); // DTM
    public final LoggableSignal loggedBridgeOutput = logged(bridgeOutput, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/BridgeOutput", bridgeOutput.getValue().name()));

    private final StatusSignal<DeviceEnableValue> deviceEnable = getDeviceEnable(false); // LESS
    public final LoggableSignal loggedDeviceEnable = logged(deviceEnable, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/DeviceEnable", deviceEnable.getValue().name()));

    private final StatusSignal<RobotEnableValue> robotEnable = getRobotEnable(false); // STANDARD
    public final LoggableSignal loggedRobotEnable = logged(robotEnable, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/RobotEnable", robotEnable.getValue().name()));

    private final StatusSignal<MotorOutputStatusValue> motorOutputStatus = getMotorOutputStatus(false); // MORE
    public final LoggableSignal loggedMotorOutputStatus = logged(motorOutputStatus, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/MotorOutputStatus", motorOutputStatus.getValue().name()));

    // --- Closed loop ---
    private final StatusSignal<Double> closedLoopReference = getClosedLoopReference(false); // STANDARD
    public final LoggableSignal loggedClosedLoopReference = logged(closedLoopReference, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopRef", closedLoopReference.getValue()));

    private final StatusSignal<Double> closedLoopError = getClosedLoopError(false); // STANDARD
    public final LoggableSignal loggedClosedLoopError = logged(closedLoopError, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopError", closedLoopError.getValue()));

    private final StatusSignal<Double> closedLoopOutput = getClosedLoopOutput(false); // DEBUG
    public final LoggableSignal loggedClosedLoopOutput = logged(closedLoopOutput, TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopOutput", closedLoopOutput.getValue()));

    private final StatusSignal<Double> closedLoopReferenceSlope = getClosedLoopReferenceSlope(false); // DEBUG
    public final LoggableSignal loggedClosedLoopReferenceSlope = logged(closedLoopReferenceSlope,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopRefSlope", closedLoopReferenceSlope.getValue()));

    private final StatusSignal<Double> closedLoopProportionalOutput = getClosedLoopProportionalOutput(false); // DTM
    public final LoggableSignal loggedClosedLoopPOutput = logged(closedLoopProportionalOutput,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopPOutput",
            closedLoopProportionalOutput.getValue()));

    private final StatusSignal<Double> closedLoopIntegratedOutput = getClosedLoopIntegratedOutput(false); // DTM
    public final LoggableSignal loggedClosedLoopIOutput = logged(closedLoopIntegratedOutput,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopIOutput",
            closedLoopIntegratedOutput.getValue()));

    private final StatusSignal<Double> closedLoopDerivativeOutput = getClosedLoopDerivativeOutput(false); // DTM
    public final LoggableSignal loggedClosedLoopDOutput = logged(closedLoopDerivativeOutput,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopDOutput",
            closedLoopDerivativeOutput.getValue()));

    private final StatusSignal<Double> closedLoopFeedForward = getClosedLoopFeedForward(false); // DEBUG
    public final LoggableSignal loggedClosedLoopFF = logged(closedLoopFeedForward, TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopFF", closedLoopFeedForward.getValue()));

    private final StatusSignal<Integer> closedLoopSlot = getClosedLoopSlot(false); // MORE
    public final LoggableSignal loggedClosedLoopSlot = logged(closedLoopSlot, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/ClosedLoopSlot", closedLoopSlot.getValue()));

    // --- Limit switches ---
    private final StatusSignal<ForwardLimitValue> forwardLimit = getForwardLimit(false); // STANDARD
    public final LoggableSignal loggedForwardLimit = logged(forwardLimit, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/ForwardLimit", isForwardLimitClosed()));

    private final StatusSignal<ReverseLimitValue> reverseLimit = getReverseLimit(false); // STANDARD
    public final LoggableSignal loggedReverseLimit = logged(reverseLimit, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/ReverseLimit", isReverseLimitClosed()));

    // --- Motion Magic ---
    private final StatusSignal<Boolean> motionMagicIsRunning = getMotionMagicIsRunning(false); // MORE
    public final LoggableSignal loggedMotionMagicIsRunning = logged(motionMagicIsRunning, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/MotionMagicIsRunning", motionMagicIsRunning.getValue()));

    private final StatusSignal<Boolean> motionMagicAtTarget = getMotionMagicAtTarget(false); // MORE
    public final LoggableSignal loggedMotionMagicAtTarget = logged(motionMagicAtTarget, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/MotionMagicAtTarget", motionMagicAtTarget.getValue()));

    // --- Individual faults ---
    private final StatusSignal<Boolean> faultBridgeBrownout = getFault_BridgeBrownout(false); // LESS
    public final LoggableSignal loggedFaultBrownout = logged(faultBridgeBrownout, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/Fault/Brownout", faultBridgeBrownout.getValue()));

    private final StatusSignal<Boolean> faultHardware = getFault_Hardware(false); // LESS
    public final LoggableSignal loggedFaultHardware = logged(faultHardware, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/Fault/Hardware", faultHardware.getValue()));

    private final StatusSignal<Boolean> faultBootDuringEnable = getFault_BootDuringEnable(false); // DEBUG
    public final LoggableSignal loggedFaultBootDuringEnable = logged(faultBootDuringEnable,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/BootDuringEnable", faultBootDuringEnable.getValue()));

    private final StatusSignal<Boolean> faultDeviceTemp = getFault_DeviceTemp(false); // LESS
    public final LoggableSignal loggedFaultDeviceTemp = logged(faultDeviceTemp, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/Fault/DeviceTemp", faultDeviceTemp.getValue()));

    private final StatusSignal<Boolean> faultProcTemp = getFault_ProcTemp(false); // LESS
    public final LoggableSignal loggedFaultProcTemp = logged(faultProcTemp, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/Fault/ProcTemp", faultProcTemp.getValue()));

    private final StatusSignal<Boolean> faultUndervoltage = getFault_Undervoltage(false); // LESS
    public final LoggableSignal loggedFaultUndervoltage = logged(faultUndervoltage, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/Fault/Undervoltage", faultUndervoltage.getValue()));

    private final StatusSignal<Boolean> faultOverSupplyV = getFault_OverSupplyV(false); // LESS
    public final LoggableSignal loggedFaultOverSupplyV = logged(faultOverSupplyV, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/Fault/OverSupplyV", faultOverSupplyV.getValue()));

    private final StatusSignal<Boolean> faultUnstableSupplyV = getFault_UnstableSupplyV(false); // LESS
    public final LoggableSignal loggedFaultUnstableSupplyV = logged(faultUnstableSupplyV, TelemetryLevel.LESS,
        () -> Logger.recordOutput(logPrefix + "/Fault/UnstableSupplyV", faultUnstableSupplyV.getValue()));

    private final StatusSignal<Boolean> faultStatorCurrLimit = getFault_StatorCurrLimit(false); // DEBUG
    public final LoggableSignal loggedFaultStatorCurrLimit = logged(faultStatorCurrLimit, TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/StatorCurrLimit", faultStatorCurrLimit.getValue()));

    private final StatusSignal<Boolean> faultSupplyCurrLimit = getFault_SupplyCurrLimit(false); // DEBUG
    public final LoggableSignal loggedFaultSupplyCurrLimit = logged(faultSupplyCurrLimit, TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/SupplyCurrLimit", faultSupplyCurrLimit.getValue()));

    private final StatusSignal<Boolean> faultForwardHardLimit = getFault_ForwardHardLimit(false); // MORE
    public final LoggableSignal loggedFaultForwardHardLimit = logged(faultForwardHardLimit, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/Fault/ForwardHardLimit", faultForwardHardLimit.getValue()));

    private final StatusSignal<Boolean> faultReverseHardLimit = getFault_ReverseHardLimit(false); // MORE
    public final LoggableSignal loggedFaultReverseHardLimit = logged(faultReverseHardLimit, TelemetryLevel.MORE,
        () -> Logger.recordOutput(logPrefix + "/Fault/ReverseHardLimit", faultReverseHardLimit.getValue()));

    private final StatusSignal<Boolean> faultForwardSoftLimit = getFault_ForwardSoftLimit(false); // STANDARD
    public final LoggableSignal loggedFaultForwardSoftLimit = logged(faultForwardSoftLimit, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/Fault/ForwardSoftLimit", faultForwardSoftLimit.getValue()));

    private final StatusSignal<Boolean> faultReverseSoftLimit = getFault_ReverseSoftLimit(false); // STANDARD
    public final LoggableSignal loggedFaultReverseSoftLimit = logged(faultReverseSoftLimit, TelemetryLevel.STANDARD,
        () -> Logger.recordOutput(logPrefix + "/Fault/ReverseSoftLimit", faultReverseSoftLimit.getValue()));

    private final StatusSignal<Boolean> faultStaticBrakeDisabled = getFault_StaticBrakeDisabled(false); // DTM
    public final LoggableSignal loggedFaultStaticBrakeDisabled = logged(faultStaticBrakeDisabled,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/Fault/StaticBrakeDisabled",
            faultStaticBrakeDisabled.getValue()));

    private final StatusSignal<Boolean> faultFusedSensorOutOfSync = getFault_FusedSensorOutOfSync(false); // DEBUG
    public final LoggableSignal loggedFaultFusedSensorOutOfSync = logged(faultFusedSensorOutOfSync,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/FusedSensorOutOfSync",
            faultFusedSensorOutOfSync.getValue()));

    private final StatusSignal<Boolean> faultRemoteSensorDataInvalid = getFault_RemoteSensorDataInvalid(false); // DEBUG
    public final LoggableSignal loggedFaultRemoteSensorDataInvalid = logged(faultRemoteSensorDataInvalid,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/RemoteSensorDataInvalid",
            faultRemoteSensorDataInvalid.getValue()));

    private final StatusSignal<Boolean> faultRemoteSensorPosOverflow = getFault_RemoteSensorPosOverflow(false); // DEBUG
    public final LoggableSignal loggedFaultRemoteSensorPosOverflow = logged(faultRemoteSensorPosOverflow,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/RemoteSensorPosOverflow",
            faultRemoteSensorPosOverflow.getValue()));

    private final StatusSignal<Boolean> faultRemoteSensorReset = getFault_RemoteSensorReset(false); // DEBUG
    public final LoggableSignal loggedFaultRemoteSensorReset = logged(faultRemoteSensorReset,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/RemoteSensorReset",
            faultRemoteSensorReset.getValue()));

    private final StatusSignal<Boolean> faultMissingHardLimitRemote = getFault_MissingHardLimitRemote(false); // DEBUG
    public final LoggableSignal loggedFaultMissingHardLimitRemote = logged(faultMissingHardLimitRemote,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/MissingHardLimitRemote",
            faultMissingHardLimitRemote.getValue()));

    private final StatusSignal<Boolean> faultMissingSoftLimitRemote = getFault_MissingSoftLimitRemote(false); // DEBUG
    public final LoggableSignal loggedFaultMissingSoftLimitRemote = logged(faultMissingSoftLimitRemote,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/MissingSoftLimitRemote",
            faultMissingSoftLimitRemote.getValue()));

    private final StatusSignal<Boolean> faultUnlicensedFeatureInUse = getFault_UnlicensedFeatureInUse(false); // DEBUG
    public final LoggableSignal loggedFaultUnlicensedFeatureInUse = logged(faultUnlicensedFeatureInUse,
        TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/UnlicensedFeatureInUse",
            faultUnlicensedFeatureInUse.getValue()));

    private final StatusSignal<Boolean> faultUsingFusedCANcoderWhileUnlicensed = getFault_UsingFusedCANcoderWhileUnlicensed(false); // DEBUG
    public final LoggableSignal loggedFaultUsingFusedCANcoderWhileUnlicensed = logged(
        faultUsingFusedCANcoderWhileUnlicensed, TelemetryLevel.DEBUG,
        () -> Logger.recordOutput(logPrefix + "/Fault/UsingFusedCANcoderWhileUnlicensed",
            faultUsingFusedCANcoderWhileUnlicensed.getValue()));

    // --- Sticky faults ---
    private final StatusSignal<Boolean> stickyFaultBridgeBrownout = getStickyFault_BridgeBrownout(false); // DTM
    public final LoggableSignal loggedStickyFaultBrownout = logged(stickyFaultBridgeBrownout,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/Brownout",
            stickyFaultBridgeBrownout.getValue()));

    private final StatusSignal<Boolean> stickyFaultHardware = getStickyFault_Hardware(false); // DTM
    public final LoggableSignal loggedStickyFaultHardware = logged(stickyFaultHardware, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/Hardware", stickyFaultHardware.getValue()));

    private final StatusSignal<Boolean> stickyFaultBootDuringEnable = getStickyFault_BootDuringEnable(false); // DTM
    public final LoggableSignal loggedStickyFaultBootDuringEnable = logged(stickyFaultBootDuringEnable,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/BootDuringEnable",
            stickyFaultBootDuringEnable.getValue()));

    private final StatusSignal<Boolean> stickyFaultDeviceTemp = getStickyFault_DeviceTemp(false); // DTM
    public final LoggableSignal loggedStickyFaultDeviceTemp = logged(stickyFaultDeviceTemp,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/DeviceTemp",
            stickyFaultDeviceTemp.getValue()));

    private final StatusSignal<Boolean> stickyFaultProcTemp = getStickyFault_ProcTemp(false); // DTM
    public final LoggableSignal loggedStickyFaultProcTemp = logged(stickyFaultProcTemp, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/ProcTemp", stickyFaultProcTemp.getValue()));

    private final StatusSignal<Boolean> stickyFaultUndervoltage = getStickyFault_Undervoltage(false); // DTM
    public final LoggableSignal loggedStickyFaultUndervoltage = logged(stickyFaultUndervoltage,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/Undervoltage",
            stickyFaultUndervoltage.getValue()));

    private final StatusSignal<Boolean> stickyFaultOverSupplyV = getStickyFault_OverSupplyV(false); // DTM
    public final LoggableSignal loggedStickyFaultOverSupplyV = logged(stickyFaultOverSupplyV,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/OverSupplyV",
            stickyFaultOverSupplyV.getValue()));

    private final StatusSignal<Boolean> stickyFaultUnstableSupplyV = getStickyFault_UnstableSupplyV(false); // DTM
    public final LoggableSignal loggedStickyFaultUnstableSupplyV = logged(stickyFaultUnstableSupplyV,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/UnstableSupplyV",
            stickyFaultUnstableSupplyV.getValue()));

    private final StatusSignal<Boolean> stickyFaultStatorCurrLimit = getStickyFault_StatorCurrLimit(false); // DTM
    public final LoggableSignal loggedStickyFaultStatorCurrLimit = logged(stickyFaultStatorCurrLimit,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/StatorCurrLimit",
            stickyFaultStatorCurrLimit.getValue()));

    private final StatusSignal<Boolean> stickyFaultSupplyCurrLimit = getStickyFault_SupplyCurrLimit(false); // DTM
    public final LoggableSignal loggedStickyFaultSupplyCurrLimit = logged(stickyFaultSupplyCurrLimit,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/SupplyCurrLimit",
            stickyFaultSupplyCurrLimit.getValue()));

    private final StatusSignal<Boolean> stickyFaultForwardHardLimit = getStickyFault_ForwardHardLimit(false); // DTM
    public final LoggableSignal loggedStickyFaultForwardHardLimit = logged(stickyFaultForwardHardLimit,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/ForwardHardLimit",
            stickyFaultForwardHardLimit.getValue()));

    private final StatusSignal<Boolean> stickyFaultReverseHardLimit = getStickyFault_ReverseHardLimit(false); // DTM
    public final LoggableSignal loggedStickyFaultReverseHardLimit = logged(stickyFaultReverseHardLimit,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/ReverseHardLimit",
            stickyFaultReverseHardLimit.getValue()));

    private final StatusSignal<Boolean> stickyFaultForwardSoftLimit = getStickyFault_ForwardSoftLimit(false); // DTM
    public final LoggableSignal loggedStickyFaultForwardSoftLimit = logged(stickyFaultForwardSoftLimit,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/ForwardSoftLimit",
            stickyFaultForwardSoftLimit.getValue()));

    private final StatusSignal<Boolean> stickyFaultReverseSoftLimit = getStickyFault_ReverseSoftLimit(false); // DTM
    public final LoggableSignal loggedStickyFaultReverseSoftLimit = logged(stickyFaultReverseSoftLimit,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/ReverseSoftLimit",
            stickyFaultReverseSoftLimit.getValue()));

    private final StatusSignal<Boolean> stickyFaultStaticBrakeDisabled = getStickyFault_StaticBrakeDisabled(false); // DTM
    public final LoggableSignal loggedStickyFaultStaticBrakeDisabled = logged(
        stickyFaultStaticBrakeDisabled, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/StaticBrakeDisabled",
            stickyFaultStaticBrakeDisabled.getValue()));

    private final StatusSignal<Boolean> stickyFaultFusedSensorOutOfSync = getStickyFault_FusedSensorOutOfSync(false); // DTM
    public final LoggableSignal loggedStickyFaultFusedSensorOutOfSync = logged(
        stickyFaultFusedSensorOutOfSync, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/FusedSensorOutOfSync",
            stickyFaultFusedSensorOutOfSync.getValue()));

    private final StatusSignal<Boolean> stickyFaultRemoteSensorDataInvalid = getStickyFault_RemoteSensorDataInvalid(false); // DTM
    public final LoggableSignal loggedStickyFaultRemoteSensorDataInvalid = logged(
        stickyFaultRemoteSensorDataInvalid, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/RemoteSensorDataInvalid",
            stickyFaultRemoteSensorDataInvalid.getValue()));

    private final StatusSignal<Boolean> stickyFaultRemoteSensorPosOverflow = getStickyFault_RemoteSensorPosOverflow(false); // DTM
    public final LoggableSignal loggedStickyFaultRemoteSensorPosOverflow = logged(
        stickyFaultRemoteSensorPosOverflow, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/RemoteSensorPosOverflow",
            stickyFaultRemoteSensorPosOverflow.getValue()));

    private final StatusSignal<Boolean> stickyFaultRemoteSensorReset = getStickyFault_RemoteSensorReset(false); // DTM
    public final LoggableSignal loggedStickyFaultRemoteSensorReset = logged(stickyFaultRemoteSensorReset,
        TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/RemoteSensorReset",
            stickyFaultRemoteSensorReset.getValue()));

    private final StatusSignal<Boolean> stickyFaultMissingDifferentialFX = getStickyFault_MissingDifferentialFX(false); // DTM
    public final LoggableSignal loggedStickyFaultMissingDifferentialFX = logged(
        stickyFaultMissingDifferentialFX, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/MissingDifferentialFX",
            stickyFaultMissingDifferentialFX.getValue()));

    private final StatusSignal<Boolean> stickyFaultMissingHardLimitRemote = getStickyFault_MissingHardLimitRemote(false); // DTM
    public final LoggableSignal loggedStickyFaultMissingHardLimitRemote = logged(
        stickyFaultMissingHardLimitRemote, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/MissingHardLimitRemote",
            stickyFaultMissingHardLimitRemote.getValue()));

    private final StatusSignal<Boolean> stickyFaultMissingSoftLimitRemote = getStickyFault_MissingSoftLimitRemote(false); // DTM
    public final LoggableSignal loggedStickyFaultMissingSoftLimitRemote = logged(
        stickyFaultMissingSoftLimitRemote, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/MissingSoftLimitRemote",
            stickyFaultMissingSoftLimitRemote.getValue()));

    private final StatusSignal<Boolean> stickyFaultUnlicensedFeatureInUse = getStickyFault_UnlicensedFeatureInUse(false); // DTM
    public final LoggableSignal loggedStickyFaultUnlicensedFeatureInUse = logged(
        stickyFaultUnlicensedFeatureInUse, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/UnlicensedFeatureInUse",
            stickyFaultUnlicensedFeatureInUse.getValue()));

    private final StatusSignal<Boolean> stickyFaultUsingFusedCANcoderWhileUnlicensed = getStickyFault_UsingFusedCANcoderWhileUnlicensed(false); // DTM
    public final LoggableSignal loggedStickyFaultUsingFusedCANcoderWhileUnlicensed = logged(
        stickyFaultUsingFusedCANcoderWhileUnlicensed, TelemetryLevel.DTM,
        () -> Logger.recordOutput(logPrefix + "/StickyFault/UsingFusedCANcoderWhileUnlicensed",
            stickyFaultUsingFusedCANcoderWhileUnlicensed.getValue()));

    private Per<TorqueUnit, CurrentUnit> kt;
    private Per<AngularVelocityUnit, VoltageUnit> kv;
    private Current motorStallCurrent;
    private boolean isProLicensed;

    private Trigger clearFaults;

    // if no name set set to motor[deviceID]
    public LoggedTalon(int deviceId, CANBus canBus) {
        this(deviceId, canBus, ("motor" + deviceId));
    }

    public LoggedTalon(int deviceId, CANBus canBus, String dashboardKey) {
        super(deviceId, canBus);
        // the replace methods are used in case the dashboardKey gives funky results
        this.logPrefix = "TalonFX/" + dashboardKey.replace("/", "_").replace(" ", "_");

        kt = getMotorKT().getValue();
        kv = getMotorKV().getValue();
        motorStallCurrent = getMotorStallCurrent().getValue();
        isProLicensed = getIsProLicensed().getValue();

        initSignalsToLog();

        String clearFaultsKey = dashboardKey + "/clearStickyFaults";
        SmartDashboard.setDefaultBoolean(clearFaultsKey, false);
        clearFaults = new Trigger(() -> SmartDashboard.getBoolean(clearFaultsKey, false));
        Command clearFaultsCommand = Commands.runOnce(() -> {
            clearStickyFaults();
            SmartDashboard.putBoolean(clearFaultsKey, false);
        });
        clearFaultsCommand.ignoringDisable(true);
        clearFaults.onTrue(clearFaultsCommand);
    }

    /** Populate signalsToLog from allSignals, filtered by the current telemetry level. */
    private void initSignalsToLog() {
        for (LoggableSignal ls : allSignals) {
            if (telemetryLevel.includes(ls.level)) {
                signalsToLog.add(ls);
            }
        }
    }

    /**
     * Refresh all signals in {@link #signalsToLog} from CAN.
     * The set is pre-filtered by telemetry level in {@link #initSignalsToLog()}.
     *
     * @return Status of the CAN refresh transaction.
     */
    private StatusCode refreshSignals() {
        if (signalsToLog.isEmpty())
            return StatusCode.OK;
        BaseStatusSignal[] signals = signalsToLog.stream()
            .map(ls -> ls.signal)
            .toArray(BaseStatusSignal[]::new);
        return BaseStatusSignal.refreshAll(signals);
    }

    /**
     * Publish the latest signal values to AdvantageKit (rate limited by
     * configuration). These values can be sent to network tables and WPILOG
     * simultaneously by configuring data receivers in Robot.
     */
    public void updateDashboard() {
        refreshSignals();
        for (LoggableSignal ls : signalsToLog) {
            ls.logger.run();
        }
    }

    public boolean isForwardLimitClosed() {
        return forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean isReverseLimitClosed() {
        return reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
    }

    // torque = kt * torqueCurrent
    public Torque getTorque() {
        // kt = torque constant
        Current tCurrent = torqueCurrent.getValue();
        return (Torque) kt.timesDivisor(tCurrent);
    }

    public Per<TorqueUnit, CurrentUnit> getMotorKtConstant() {
        return kt;
    }

    public Per<AngularVelocityUnit, VoltageUnit> getMotorKvConstant() {
        return kv;
    }

    public Current getMotorStallCurrentConstant() {
        return motorStallCurrent;
    }

    public boolean getIsProLicensedConstant() {
        return isProLicensed;
    }

}
