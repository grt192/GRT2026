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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class LoggedTalon extends TalonFX {
    public enum TelemetryLevel {
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
    public static class LoggableSignal {
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

    private TelemetryLevel telemetryLevel = DEFAULT_TELEMETRY_LEVEL;

    /** All LoggableSignals, populated during {@link #registerSignals()}. */
    private final List<LoggableSignal> allSignals = new ArrayList<>();

    /** Curated set of signals to refresh and log, filtered by telemetry level. */
    private final Set<LoggableSignal> signalsToLog = new LinkedHashSet<>();

    /** Cached array of signals for {@link #refreshSignals()}, rebuilt when signalsToLog changes. */
    private BaseStatusSignal[] cachedSignalArray = new BaseStatusSignal[0];

    /** Lock guarding {@link #signalsToLog} and {@link #cachedSignalArray}. */
    private final Object signalLock = new Object();

    /** Creates a LoggableSignal, registers it in {@link #allSignals}, and returns it. */
    private LoggableSignal logged(BaseStatusSignal signal, TelemetryLevel level, Runnable logger) {
        LoggableSignal ls = new LoggableSignal(signal, level, logger);
        allSignals.add(ls);
        return ls;
    }

    /** Add a LoggableSignal to the active logging set. */
    public void addToLogging(LoggableSignal ls) {
        synchronized (signalLock) {
            signalsToLog.add(ls);
            rebuildCachedSignalArray();
        }
    }

    public void removeFromLogging(LoggableSignal ls) {
        synchronized (signalLock) {
            signalsToLog.remove(ls);
            rebuildCachedSignalArray();
        }
    }

    private void rebuildCachedSignalArray() {
        cachedSignalArray = signalsToLog.stream()
            .map(ls -> ls.signal)
            .toArray(BaseStatusSignal[]::new);
    }

    public void setTelemetryLevel(TelemetryLevel level) {
        telemetryLevel = level;
        populateSignalsToLog();
    }

    // ===== Cached signals (refresh once, then read many) =====
    // --- Position / Velocity / Acceleration ---
    private final StatusSignal<Angle> position = getPosition(false);
    public LoggableSignal loggedPosition;

    private final StatusSignal<AngularVelocity> velocity = getVelocity(false);
    public LoggableSignal loggedVelocity;

    private final StatusSignal<AngularAcceleration> acceleration = getAcceleration(false);
    public LoggableSignal loggedAcceleration;

    private final StatusSignal<Angle> rotorPosition = getRotorPosition(false);
    public LoggableSignal loggedRotorPosition;

    private final StatusSignal<AngularVelocity> rotorVelocity = getRotorVelocity(false);
    public LoggableSignal loggedRotorVelocity;

    // --- Duty cycle / Voltage ---
    private final StatusSignal<Double> dutyCycle = getDutyCycle(false);
    public LoggableSignal loggedDutyCycle;

    private final StatusSignal<Voltage> supplyVoltage = getSupplyVoltage(false);
    public LoggableSignal loggedSupplyVoltage;

    private final StatusSignal<Voltage> appliedVoltage = getMotorVoltage(false);
    public LoggableSignal loggedAppliedVoltage;

    // --- Current ---
    private final StatusSignal<Current> supplyCurrent = getSupplyCurrent(false);
    public LoggableSignal loggedSupplyCurrent;

    private final StatusSignal<Current> statorCurrent = getStatorCurrent(false);
    public LoggableSignal loggedStatorCurrent;

    private final StatusSignal<Current> torqueCurrent = getTorqueCurrent(false);
    public LoggableSignal loggedTorqueCurrent;

    // --- Temperature ---
    private final StatusSignal<Temperature> deviceTemp = getDeviceTemp(false);
    public LoggableSignal loggedDeviceTemp;

    private final StatusSignal<Temperature> processorTemp = getProcessorTemp(false);
    public LoggableSignal loggedProcessorTemp;

    // --- Control mode / output ---
    private final StatusSignal<ControlModeValue> controlMode = getControlMode(false);
    public LoggableSignal loggedControlMode;

    private final StatusSignal<AppliedRotorPolarityValue> appliedRotorPolarity = getAppliedRotorPolarity(false);
    public LoggableSignal loggedAppliedRotorPolarity;

    private final StatusSignal<BridgeOutputValue> bridgeOutput = getBridgeOutput(false);
    public LoggableSignal loggedBridgeOutput;

    private final StatusSignal<DeviceEnableValue> deviceEnable = getDeviceEnable(false);
    public LoggableSignal loggedDeviceEnable;

    private final StatusSignal<RobotEnableValue> robotEnable = getRobotEnable(false);
    public LoggableSignal loggedRobotEnable;

    private final StatusSignal<MotorOutputStatusValue> motorOutputStatus = getMotorOutputStatus(false);
    public LoggableSignal loggedMotorOutputStatus;

    // --- Closed loop ---
    private final StatusSignal<Double> closedLoopReference = getClosedLoopReference(false);
    public LoggableSignal loggedClosedLoopReference;

    private final StatusSignal<Double> closedLoopError = getClosedLoopError(false);
    public LoggableSignal loggedClosedLoopError;

    private final StatusSignal<Double> closedLoopOutput = getClosedLoopOutput(false);
    public LoggableSignal loggedClosedLoopOutput;

    private final StatusSignal<Double> closedLoopReferenceSlope = getClosedLoopReferenceSlope(false);
    public LoggableSignal loggedClosedLoopReferenceSlope;

    private final StatusSignal<Double> closedLoopProportionalOutput = getClosedLoopProportionalOutput(false);
    public LoggableSignal loggedClosedLoopPOutput;

    private final StatusSignal<Double> closedLoopIntegratedOutput = getClosedLoopIntegratedOutput(false);
    public LoggableSignal loggedClosedLoopIOutput;

    private final StatusSignal<Double> closedLoopDerivativeOutput = getClosedLoopDerivativeOutput(false);
    public LoggableSignal loggedClosedLoopDOutput;

    private final StatusSignal<Double> closedLoopFeedForward = getClosedLoopFeedForward(false);
    public LoggableSignal loggedClosedLoopFF;

    private final StatusSignal<Integer> closedLoopSlot = getClosedLoopSlot(false);
    public LoggableSignal loggedClosedLoopSlot;

    // --- Limit switches ---
    private final StatusSignal<ForwardLimitValue> forwardLimit = getForwardLimit(false);
    public LoggableSignal loggedForwardLimit;

    private final StatusSignal<ReverseLimitValue> reverseLimit = getReverseLimit(false);
    public LoggableSignal loggedReverseLimit;

    // --- Motion Magic ---
    private final StatusSignal<Boolean> motionMagicIsRunning = getMotionMagicIsRunning(false);
    public LoggableSignal loggedMotionMagicIsRunning;

    private final StatusSignal<Boolean> motionMagicAtTarget = getMotionMagicAtTarget(false);
    public LoggableSignal loggedMotionMagicAtTarget;

    // --- Individual faults ---
    private final StatusSignal<Boolean> faultBridgeBrownout = getFault_BridgeBrownout(false);
    public LoggableSignal loggedFaultBrownout;

    private final StatusSignal<Boolean> faultHardware = getFault_Hardware(false);
    public LoggableSignal loggedFaultHardware;

    private final StatusSignal<Boolean> faultBootDuringEnable = getFault_BootDuringEnable(false);
    public LoggableSignal loggedFaultBootDuringEnable;

    private final StatusSignal<Boolean> faultDeviceTemp = getFault_DeviceTemp(false);
    public LoggableSignal loggedFaultDeviceTemp;

    private final StatusSignal<Boolean> faultProcTemp = getFault_ProcTemp(false);
    public LoggableSignal loggedFaultProcTemp;

    private final StatusSignal<Boolean> faultUndervoltage = getFault_Undervoltage(false);
    public LoggableSignal loggedFaultUndervoltage;

    private final StatusSignal<Boolean> faultOverSupplyV = getFault_OverSupplyV(false);
    public LoggableSignal loggedFaultOverSupplyV;

    private final StatusSignal<Boolean> faultUnstableSupplyV = getFault_UnstableSupplyV(false);
    public LoggableSignal loggedFaultUnstableSupplyV;

    private final StatusSignal<Boolean> faultStatorCurrLimit = getFault_StatorCurrLimit(false);
    public LoggableSignal loggedFaultStatorCurrLimit;

    private final StatusSignal<Boolean> faultSupplyCurrLimit = getFault_SupplyCurrLimit(false);
    public LoggableSignal loggedFaultSupplyCurrLimit;

    private final StatusSignal<Boolean> faultForwardHardLimit = getFault_ForwardHardLimit(false);
    public LoggableSignal loggedFaultForwardHardLimit;

    private final StatusSignal<Boolean> faultReverseHardLimit = getFault_ReverseHardLimit(false);
    public LoggableSignal loggedFaultReverseHardLimit;

    private final StatusSignal<Boolean> faultForwardSoftLimit = getFault_ForwardSoftLimit(false);
    public LoggableSignal loggedFaultForwardSoftLimit;

    private final StatusSignal<Boolean> faultReverseSoftLimit = getFault_ReverseSoftLimit(false);
    public LoggableSignal loggedFaultReverseSoftLimit;

    private final StatusSignal<Boolean> faultStaticBrakeDisabled = getFault_StaticBrakeDisabled(false);
    public LoggableSignal loggedFaultStaticBrakeDisabled;

    private final StatusSignal<Boolean> faultFusedSensorOutOfSync = getFault_FusedSensorOutOfSync(false);
    public LoggableSignal loggedFaultFusedSensorOutOfSync;

    private final StatusSignal<Boolean> faultRemoteSensorDataInvalid = getFault_RemoteSensorDataInvalid(false);
    public LoggableSignal loggedFaultRemoteSensorDataInvalid;

    private final StatusSignal<Boolean> faultRemoteSensorPosOverflow = getFault_RemoteSensorPosOverflow(false);
    public LoggableSignal loggedFaultRemoteSensorPosOverflow;

    private final StatusSignal<Boolean> faultRemoteSensorReset = getFault_RemoteSensorReset(false);
    public LoggableSignal loggedFaultRemoteSensorReset;

    private final StatusSignal<Boolean> faultMissingHardLimitRemote = getFault_MissingHardLimitRemote(false);
    public LoggableSignal loggedFaultMissingHardLimitRemote;

    private final StatusSignal<Boolean> faultMissingSoftLimitRemote = getFault_MissingSoftLimitRemote(false);
    public LoggableSignal loggedFaultMissingSoftLimitRemote;

    private final StatusSignal<Boolean> faultUnlicensedFeatureInUse = getFault_UnlicensedFeatureInUse(false);
    public LoggableSignal loggedFaultUnlicensedFeatureInUse;

    private final StatusSignal<Boolean> faultUsingFusedCANcoderWhileUnlicensed = getFault_UsingFusedCANcoderWhileUnlicensed(false);
    public LoggableSignal loggedFaultUsingFusedCANcoderWhileUnlicensed;

    // --- Sticky faults ---
    private final StatusSignal<Boolean> stickyFaultBridgeBrownout = getStickyFault_BridgeBrownout(false);
    public LoggableSignal loggedStickyFaultBrownout;

    private final StatusSignal<Boolean> stickyFaultHardware = getStickyFault_Hardware(false);
    public LoggableSignal loggedStickyFaultHardware;

    private final StatusSignal<Boolean> stickyFaultBootDuringEnable = getStickyFault_BootDuringEnable(false);
    public LoggableSignal loggedStickyFaultBootDuringEnable;

    private final StatusSignal<Boolean> stickyFaultDeviceTemp = getStickyFault_DeviceTemp(false);
    public LoggableSignal loggedStickyFaultDeviceTemp;

    private final StatusSignal<Boolean> stickyFaultProcTemp = getStickyFault_ProcTemp(false);
    public LoggableSignal loggedStickyFaultProcTemp;

    private final StatusSignal<Boolean> stickyFaultUndervoltage = getStickyFault_Undervoltage(false);
    public LoggableSignal loggedStickyFaultUndervoltage;

    private final StatusSignal<Boolean> stickyFaultOverSupplyV = getStickyFault_OverSupplyV(false);
    public LoggableSignal loggedStickyFaultOverSupplyV;

    private final StatusSignal<Boolean> stickyFaultUnstableSupplyV = getStickyFault_UnstableSupplyV(false);
    public LoggableSignal loggedStickyFaultUnstableSupplyV;

    private final StatusSignal<Boolean> stickyFaultStatorCurrLimit = getStickyFault_StatorCurrLimit(false);
    public LoggableSignal loggedStickyFaultStatorCurrLimit;

    private final StatusSignal<Boolean> stickyFaultSupplyCurrLimit = getStickyFault_SupplyCurrLimit(false);
    public LoggableSignal loggedStickyFaultSupplyCurrLimit;

    private final StatusSignal<Boolean> stickyFaultForwardHardLimit = getStickyFault_ForwardHardLimit(false);
    public LoggableSignal loggedStickyFaultForwardHardLimit;

    private final StatusSignal<Boolean> stickyFaultReverseHardLimit = getStickyFault_ReverseHardLimit(false);
    public LoggableSignal loggedStickyFaultReverseHardLimit;

    private final StatusSignal<Boolean> stickyFaultForwardSoftLimit = getStickyFault_ForwardSoftLimit(false);
    public LoggableSignal loggedStickyFaultForwardSoftLimit;

    private final StatusSignal<Boolean> stickyFaultReverseSoftLimit = getStickyFault_ReverseSoftLimit(false);
    public LoggableSignal loggedStickyFaultReverseSoftLimit;

    private final StatusSignal<Boolean> stickyFaultStaticBrakeDisabled = getStickyFault_StaticBrakeDisabled(false);
    public LoggableSignal loggedStickyFaultStaticBrakeDisabled;

    private final StatusSignal<Boolean> stickyFaultFusedSensorOutOfSync = getStickyFault_FusedSensorOutOfSync(false);
    public LoggableSignal loggedStickyFaultFusedSensorOutOfSync;

    private final StatusSignal<Boolean> stickyFaultRemoteSensorDataInvalid = getStickyFault_RemoteSensorDataInvalid(false);
    public LoggableSignal loggedStickyFaultRemoteSensorDataInvalid;

    private final StatusSignal<Boolean> stickyFaultRemoteSensorPosOverflow = getStickyFault_RemoteSensorPosOverflow(false);
    public LoggableSignal loggedStickyFaultRemoteSensorPosOverflow;

    private final StatusSignal<Boolean> stickyFaultRemoteSensorReset = getStickyFault_RemoteSensorReset(false);
    public LoggableSignal loggedStickyFaultRemoteSensorReset;

    private final StatusSignal<Boolean> stickyFaultMissingDifferentialFX = getStickyFault_MissingDifferentialFX(false);
    public LoggableSignal loggedStickyFaultMissingDifferentialFX;

    private final StatusSignal<Boolean> stickyFaultMissingHardLimitRemote = getStickyFault_MissingHardLimitRemote(false);
    public LoggableSignal loggedStickyFaultMissingHardLimitRemote;

    private final StatusSignal<Boolean> stickyFaultMissingSoftLimitRemote = getStickyFault_MissingSoftLimitRemote(false);
    public LoggableSignal loggedStickyFaultMissingSoftLimitRemote;

    private final StatusSignal<Boolean> stickyFaultUnlicensedFeatureInUse = getStickyFault_UnlicensedFeatureInUse(false);
    public LoggableSignal loggedStickyFaultUnlicensedFeatureInUse;

    private final StatusSignal<Boolean> stickyFaultUsingFusedCANcoderWhileUnlicensed = getStickyFault_UsingFusedCANcoderWhileUnlicensed(false);
    public LoggableSignal loggedStickyFaultUsingFusedCANcoderWhileUnlicensed;

    private final Per<TorqueUnit, CurrentUnit> kt;
    private final Per<AngularVelocityUnit, VoltageUnit> kv;
    private final Current motorStallCurrent;
    private final boolean isProLicensed;

    private Trigger clearFaults;

    // Default name is "motor{deviceID}" (no brackets)
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

        registerSignals();
        populateSignalsToLog();

        String clearFaultsKey = dashboardKey + "/clearStickyFaults";
        SmartDashboard.setDefaultBoolean(clearFaultsKey, false);
        clearFaults = new Trigger(() -> SmartDashboard.getBoolean(clearFaultsKey, false));
        // Note: this Trigger is never unregistered and will persist for the lifetime of the process.
        clearFaults.onTrue(Commands.runOnce(() -> {
            clearStickyFaults();
            SmartDashboard.putBoolean(clearFaultsKey, false);
        }).ignoringDisable(true));
    }

    /**
     * Register all LoggableSignals. Called once from the constructor after all
     * StatusSignal fields have been initialized, ensuring logPrefix and allSignals
     * are ready before any logging lambdas are created.
     */
    private void registerSignals() {
        // --- Position / Velocity / Acceleration ---
        loggedPosition = logged(position, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/Position", position.getValue().in(Units.Radians)));
        loggedVelocity = logged(velocity, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/Velocity", velocity.getValue().in(Units.RadiansPerSecond)));
        loggedAcceleration = logged(acceleration, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/Acceleration",
                acceleration.getValue().in(Units.RadiansPerSecondPerSecond)));
        loggedRotorPosition = logged(rotorPosition, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/RotorPosition", rotorPosition.getValue().in(Units.Radians)));
        loggedRotorVelocity = logged(rotorVelocity, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/RotorVelocity",
                rotorVelocity.getValue().in(Units.RadiansPerSecond)));

        // --- Duty cycle / Voltage ---
        loggedDutyCycle = logged(dutyCycle, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/DutyCycle", dutyCycle.getValue()));
        loggedSupplyVoltage = logged(supplyVoltage, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/SupplyVoltage", supplyVoltage.getValue().in(Units.Volts)));
        loggedAppliedVoltage = logged(appliedVoltage, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/AppliedVoltage", appliedVoltage.getValue().in(Units.Volts)));

        // --- Current ---
        loggedSupplyCurrent = logged(supplyCurrent, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/SupplyCurrent", supplyCurrent.getValue().in(Units.Amps)));
        loggedStatorCurrent = logged(statorCurrent, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/StatorCurrent", statorCurrent.getValue().in(Units.Amps)));
        loggedTorqueCurrent = logged(torqueCurrent, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/TorqueCurrent", torqueCurrent.getValue().in(Units.Amps)));

        // --- Temperature ---
        loggedDeviceTemp = logged(deviceTemp, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/TempC", deviceTemp.getValue().in(Units.Celsius)));
        loggedProcessorTemp = logged(processorTemp, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/ProcessorTempC", processorTemp.getValue().in(Units.Celsius)));

        // --- Control mode / output ---
        loggedControlMode = logged(controlMode, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/ControlMode", controlMode.getValue().name()));
        loggedAppliedRotorPolarity = logged(appliedRotorPolarity, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/AppliedRotorPolarity",
                appliedRotorPolarity.getValue().name()));
        loggedBridgeOutput = logged(bridgeOutput, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/BridgeOutput", bridgeOutput.getValue().name()));
        loggedDeviceEnable = logged(deviceEnable, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/DeviceEnable", deviceEnable.getValue().name()));
        loggedRobotEnable = logged(robotEnable, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/RobotEnable", robotEnable.getValue().name()));
        loggedMotorOutputStatus = logged(motorOutputStatus, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/MotorOutputStatus", motorOutputStatus.getValue().name()));

        // --- Closed loop ---
        loggedClosedLoopReference = logged(closedLoopReference, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopRef", closedLoopReference.getValue()));
        loggedClosedLoopError = logged(closedLoopError, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopError", closedLoopError.getValue()));
        loggedClosedLoopOutput = logged(closedLoopOutput, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopOutput", closedLoopOutput.getValue()));
        loggedClosedLoopReferenceSlope = logged(closedLoopReferenceSlope, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopRefSlope", closedLoopReferenceSlope.getValue()));
        loggedClosedLoopPOutput = logged(closedLoopProportionalOutput, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopPOutput",
                closedLoopProportionalOutput.getValue()));
        loggedClosedLoopIOutput = logged(closedLoopIntegratedOutput, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopIOutput",
                closedLoopIntegratedOutput.getValue()));
        loggedClosedLoopDOutput = logged(closedLoopDerivativeOutput, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopDOutput",
                closedLoopDerivativeOutput.getValue()));
        loggedClosedLoopFF = logged(closedLoopFeedForward, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopFF", closedLoopFeedForward.getValue()));
        loggedClosedLoopSlot = logged(closedLoopSlot, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/ClosedLoopSlot", closedLoopSlot.getValue()));

        // --- Limit switches ---
        loggedForwardLimit = logged(forwardLimit, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/ForwardLimit", isForwardLimitClosed()));
        loggedReverseLimit = logged(reverseLimit, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/ReverseLimit", isReverseLimitClosed()));

        // --- Motion Magic ---
        loggedMotionMagicIsRunning = logged(motionMagicIsRunning, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/MotionMagicIsRunning", motionMagicIsRunning.getValue()));
        loggedMotionMagicAtTarget = logged(motionMagicAtTarget, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/MotionMagicAtTarget", motionMagicAtTarget.getValue()));

        // --- Individual faults ---
        loggedFaultBrownout = logged(faultBridgeBrownout, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/Brownout", faultBridgeBrownout.getValue()));
        loggedFaultHardware = logged(faultHardware, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/Hardware", faultHardware.getValue()));
        loggedFaultBootDuringEnable = logged(faultBootDuringEnable, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/BootDuringEnable", faultBootDuringEnable.getValue()));
        loggedFaultDeviceTemp = logged(faultDeviceTemp, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/DeviceTemp", faultDeviceTemp.getValue()));
        loggedFaultProcTemp = logged(faultProcTemp, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/ProcTemp", faultProcTemp.getValue()));
        loggedFaultUndervoltage = logged(faultUndervoltage, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/Undervoltage", faultUndervoltage.getValue()));
        loggedFaultOverSupplyV = logged(faultOverSupplyV, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/OverSupplyV", faultOverSupplyV.getValue()));
        loggedFaultUnstableSupplyV = logged(faultUnstableSupplyV, TelemetryLevel.LESS,
            () -> Logger.recordOutput(logPrefix + "/Fault/UnstableSupplyV", faultUnstableSupplyV.getValue()));
        loggedFaultStatorCurrLimit = logged(faultStatorCurrLimit, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/StatorCurrLimit", faultStatorCurrLimit.getValue()));
        loggedFaultSupplyCurrLimit = logged(faultSupplyCurrLimit, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/SupplyCurrLimit", faultSupplyCurrLimit.getValue()));
        loggedFaultForwardHardLimit = logged(faultForwardHardLimit, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/Fault/ForwardHardLimit", faultForwardHardLimit.getValue()));
        loggedFaultReverseHardLimit = logged(faultReverseHardLimit, TelemetryLevel.MORE,
            () -> Logger.recordOutput(logPrefix + "/Fault/ReverseHardLimit", faultReverseHardLimit.getValue()));
        loggedFaultForwardSoftLimit = logged(faultForwardSoftLimit, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/Fault/ForwardSoftLimit", faultForwardSoftLimit.getValue()));
        loggedFaultReverseSoftLimit = logged(faultReverseSoftLimit, TelemetryLevel.STANDARD,
            () -> Logger.recordOutput(logPrefix + "/Fault/ReverseSoftLimit", faultReverseSoftLimit.getValue()));
        loggedFaultStaticBrakeDisabled = logged(faultStaticBrakeDisabled, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/Fault/StaticBrakeDisabled",
                faultStaticBrakeDisabled.getValue()));
        loggedFaultFusedSensorOutOfSync = logged(faultFusedSensorOutOfSync, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/FusedSensorOutOfSync",
                faultFusedSensorOutOfSync.getValue()));
        loggedFaultRemoteSensorDataInvalid = logged(faultRemoteSensorDataInvalid, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/RemoteSensorDataInvalid",
                faultRemoteSensorDataInvalid.getValue()));
        loggedFaultRemoteSensorPosOverflow = logged(faultRemoteSensorPosOverflow, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/RemoteSensorPosOverflow",
                faultRemoteSensorPosOverflow.getValue()));
        loggedFaultRemoteSensorReset = logged(faultRemoteSensorReset, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/RemoteSensorReset",
                faultRemoteSensorReset.getValue()));
        loggedFaultMissingHardLimitRemote = logged(faultMissingHardLimitRemote, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/MissingHardLimitRemote",
                faultMissingHardLimitRemote.getValue()));
        loggedFaultMissingSoftLimitRemote = logged(faultMissingSoftLimitRemote, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/MissingSoftLimitRemote",
                faultMissingSoftLimitRemote.getValue()));
        loggedFaultUnlicensedFeatureInUse = logged(faultUnlicensedFeatureInUse, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/UnlicensedFeatureInUse",
                faultUnlicensedFeatureInUse.getValue()));
        loggedFaultUsingFusedCANcoderWhileUnlicensed = logged(
            faultUsingFusedCANcoderWhileUnlicensed, TelemetryLevel.DEBUG,
            () -> Logger.recordOutput(logPrefix + "/Fault/UsingFusedCANcoderWhileUnlicensed",
                faultUsingFusedCANcoderWhileUnlicensed.getValue()));

        // --- Sticky faults ---
        loggedStickyFaultBrownout = logged(stickyFaultBridgeBrownout, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/Brownout",
                stickyFaultBridgeBrownout.getValue()));
        loggedStickyFaultHardware = logged(stickyFaultHardware, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/Hardware", stickyFaultHardware.getValue()));
        loggedStickyFaultBootDuringEnable = logged(stickyFaultBootDuringEnable, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/BootDuringEnable",
                stickyFaultBootDuringEnable.getValue()));
        loggedStickyFaultDeviceTemp = logged(stickyFaultDeviceTemp, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/DeviceTemp",
                stickyFaultDeviceTemp.getValue()));
        loggedStickyFaultProcTemp = logged(stickyFaultProcTemp, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/ProcTemp", stickyFaultProcTemp.getValue()));
        loggedStickyFaultUndervoltage = logged(stickyFaultUndervoltage, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/Undervoltage",
                stickyFaultUndervoltage.getValue()));
        loggedStickyFaultOverSupplyV = logged(stickyFaultOverSupplyV, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/OverSupplyV",
                stickyFaultOverSupplyV.getValue()));
        loggedStickyFaultUnstableSupplyV = logged(stickyFaultUnstableSupplyV, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/UnstableSupplyV",
                stickyFaultUnstableSupplyV.getValue()));
        loggedStickyFaultStatorCurrLimit = logged(stickyFaultStatorCurrLimit, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/StatorCurrLimit",
                stickyFaultStatorCurrLimit.getValue()));
        loggedStickyFaultSupplyCurrLimit = logged(stickyFaultSupplyCurrLimit, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/SupplyCurrLimit",
                stickyFaultSupplyCurrLimit.getValue()));
        loggedStickyFaultForwardHardLimit = logged(stickyFaultForwardHardLimit, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/ForwardHardLimit",
                stickyFaultForwardHardLimit.getValue()));
        loggedStickyFaultReverseHardLimit = logged(stickyFaultReverseHardLimit, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/ReverseHardLimit",
                stickyFaultReverseHardLimit.getValue()));
        loggedStickyFaultForwardSoftLimit = logged(stickyFaultForwardSoftLimit, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/ForwardSoftLimit",
                stickyFaultForwardSoftLimit.getValue()));
        loggedStickyFaultReverseSoftLimit = logged(stickyFaultReverseSoftLimit, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/ReverseSoftLimit",
                stickyFaultReverseSoftLimit.getValue()));
        loggedStickyFaultStaticBrakeDisabled = logged(stickyFaultStaticBrakeDisabled, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/StaticBrakeDisabled",
                stickyFaultStaticBrakeDisabled.getValue()));
        loggedStickyFaultFusedSensorOutOfSync = logged(stickyFaultFusedSensorOutOfSync, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/FusedSensorOutOfSync",
                stickyFaultFusedSensorOutOfSync.getValue()));
        loggedStickyFaultRemoteSensorDataInvalid = logged(stickyFaultRemoteSensorDataInvalid, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/RemoteSensorDataInvalid",
                stickyFaultRemoteSensorDataInvalid.getValue()));
        loggedStickyFaultRemoteSensorPosOverflow = logged(stickyFaultRemoteSensorPosOverflow, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/RemoteSensorPosOverflow",
                stickyFaultRemoteSensorPosOverflow.getValue()));
        loggedStickyFaultRemoteSensorReset = logged(stickyFaultRemoteSensorReset, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/RemoteSensorReset",
                stickyFaultRemoteSensorReset.getValue()));
        loggedStickyFaultMissingDifferentialFX = logged(stickyFaultMissingDifferentialFX, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/MissingDifferentialFX",
                stickyFaultMissingDifferentialFX.getValue()));
        loggedStickyFaultMissingHardLimitRemote = logged(stickyFaultMissingHardLimitRemote, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/MissingHardLimitRemote",
                stickyFaultMissingHardLimitRemote.getValue()));
        loggedStickyFaultMissingSoftLimitRemote = logged(stickyFaultMissingSoftLimitRemote, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/MissingSoftLimitRemote",
                stickyFaultMissingSoftLimitRemote.getValue()));
        loggedStickyFaultUnlicensedFeatureInUse = logged(stickyFaultUnlicensedFeatureInUse, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/UnlicensedFeatureInUse",
                stickyFaultUnlicensedFeatureInUse.getValue()));
        loggedStickyFaultUsingFusedCANcoderWhileUnlicensed = logged(
            stickyFaultUsingFusedCANcoderWhileUnlicensed, TelemetryLevel.DTM,
            () -> Logger.recordOutput(logPrefix + "/StickyFault/UsingFusedCANcoderWhileUnlicensed",
                stickyFaultUsingFusedCANcoderWhileUnlicensed.getValue()));
    }

    /** Populate signalsToLog from allSignals, filtered by the current telemetry level. */
    private void populateSignalsToLog() {
        synchronized (signalLock) {
            signalsToLog.clear();

            for (LoggableSignal ls : allSignals) {
                if (telemetryLevel.includes(ls.level)) {
                    signalsToLog.add(ls);
                }
            }
            rebuildCachedSignalArray();
        }
    }

    /**
     * Refresh all signals in {@link #signalsToLog} from CAN.
     * The set is pre-filtered by telemetry level in {@link #populateSignalsToLog()}.
     *
     * @return Status of the CAN refresh transaction.
     */
    private StatusCode refreshSignals() {
        if (cachedSignalArray.length == 0)
            return StatusCode.OK;
        return BaseStatusSignal.refreshAll(cachedSignalArray);
    }

    /**
     * Publish the latest signal values to AdvantageKit (rate limited by
     * configuration). These values can be sent to network tables and WPILOG
     * simultaneously by configuring data receivers in Robot.
     */
    public void updateDashboard() {
        synchronized (signalLock) {
            StatusCode status = refreshSignals();
            if (!status.isOK()) {
                Logger.recordOutput(logPrefix + "/RefreshStatus", status.getName());
            }
            for (LoggableSignal ls : signalsToLog) {
                ls.logger.run();
            }
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
