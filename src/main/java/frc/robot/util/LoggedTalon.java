package frc.robot.util;

import static edu.wpi.first.units.Units.NewtonMeters;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
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


public class LoggedTalon extends TalonFX {
  public enum TelemetryLevel {
    LESS,
    STANDARD,
    MORE,
    DEBUG,
    DTM;

    public boolean includes(TelemetryLevel required) {
      return this.ordinal() >= required.ordinal();
    }
  }

  private static final Consumer<String> nullConsumer = n -> {
  };

  private static final TelemetryLevel DEFAULT_TELEMETRY_LEVEL = TelemetryLevel.STANDARD;
  private final String logPrefix;

  public TelemetryLevel telemetryLevel = DEFAULT_TELEMETRY_LEVEL;

  // ===== Cached signals (refresh once, then read many) =====

  // --- Position / Velocity / Acceleration ---
  public final LeveledSignal<Angle> position = new LeveledSignal<>(getPosition(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<AngularVelocity> velocity = new LeveledSignal<>(getVelocity(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<AngularAcceleration> acceleration = new LeveledSignal<>(getAcceleration(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Angle> rotorPosition = new LeveledSignal<>(getRotorPosition(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<AngularVelocity> rotorVelocity = new LeveledSignal<>(getRotorVelocity(false), TelemetryLevel.MORE, nullConsumer);

  // --- Duty cycle / Voltage ---
  public final LeveledSignal<Double> dutyCycle = new LeveledSignal<>(getDutyCycle(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<Voltage> supplyVoltage = new LeveledSignal<>(getSupplyVoltage(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<Voltage> motorVoltage = new LeveledSignal<>(getMotorVoltage(false), TelemetryLevel.STANDARD, nullConsumer);

  // --- Current ---
  public final LeveledSignal<Current> supplyCurrent = new LeveledSignal<>(getSupplyCurrent(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Current> statorCurrent = new LeveledSignal<>(getStatorCurrent(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<Current> torqueCurrent = new LeveledSignal<>(getTorqueCurrent(false), TelemetryLevel.STANDARD, nullConsumer);

  // --- Temperature ---
  public final LeveledSignal<Temperature> deviceTemp = new LeveledSignal<>(getDeviceTemp(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Temperature> processorTemp = new LeveledSignal<>(getProcessorTemp(false), TelemetryLevel.MORE, nullConsumer);

  // --- Control mode / output ---
  public final LeveledSignal<ControlModeValue> controlMode = new LeveledSignal<>(getControlMode(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<AppliedRotorPolarityValue> appliedRotorPolarity = new LeveledSignal<>(getAppliedRotorPolarity(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<BridgeOutputValue> bridgeOutput = new LeveledSignal<>(getBridgeOutput(false), TelemetryLevel.DTM, nullConsumer);
  public final LeveledSignal<DeviceEnableValue> deviceEnable = new LeveledSignal<>(getDeviceEnable(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<RobotEnableValue> robotEnable = new LeveledSignal<>(getRobotEnable(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<MotorOutputStatusValue> motorOutputStatus = new LeveledSignal<>(getMotorOutputStatus(false), TelemetryLevel.MORE, nullConsumer);

  // --- Closed loop ---
  public final LeveledSignal<Double> closedLoopReference = new LeveledSignal<>(getClosedLoopReference(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<Double> closedLoopError = new LeveledSignal<>(getClosedLoopError(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Double> closedLoopOutput = new LeveledSignal<>(getClosedLoopOutput(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Double> closedLoopReferenceSlope = new LeveledSignal<>(getClosedLoopReferenceSlope(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Double> closedLoopProportionalOutput = new LeveledSignal<>(getClosedLoopProportionalOutput(false), TelemetryLevel.DTM, nullConsumer);
  public final LeveledSignal<Double> closedLoopIntegratedOutput = new LeveledSignal<>(getClosedLoopIntegratedOutput(false), TelemetryLevel.DTM, nullConsumer);
  public final LeveledSignal<Double> closedLoopDerivativeOutput = new LeveledSignal<>(getClosedLoopDerivativeOutput(false), TelemetryLevel.DTM, nullConsumer);
  public final LeveledSignal<Double> closedLoopFeedForward = new LeveledSignal<>(getClosedLoopFeedForward(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Integer> closedLoopSlot = new LeveledSignal<>(getClosedLoopSlot(false), TelemetryLevel.MORE, nullConsumer);

  // --- Limit switches ---
  public final LeveledSignal<ForwardLimitValue> forwardLimit = new LeveledSignal<>(getForwardLimit(false), TelemetryLevel.STANDARD, nullConsumer);
  public final LeveledSignal<ReverseLimitValue> reverseLimit = new LeveledSignal<>(getReverseLimit(false), TelemetryLevel.STANDARD, nullConsumer);

  // --- Motion Magic ---
  public final LeveledSignal<Boolean> motionMagicIsRunning = new LeveledSignal<>(getMotionMagicIsRunning(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> motionMagicAtTarget = new LeveledSignal<>(getMotionMagicAtTarget(false), TelemetryLevel.MORE, nullConsumer);

  // --- Individual faults ---
  public final LeveledSignal<Boolean> faultBridgeBrownout = new LeveledSignal<>(getFault_BridgeBrownout(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> faultHardware = new LeveledSignal<>(getFault_Hardware(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> faultBootDuringEnable = new LeveledSignal<>(getFault_BootDuringEnable(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultDeviceTemp = new LeveledSignal<>(getFault_DeviceTemp(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> faultProcTemp = new LeveledSignal<>(getFault_ProcTemp(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> faultUndervoltage = new LeveledSignal<>(getFault_Undervoltage(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> faultOverSupplyV = new LeveledSignal<>(getFault_OverSupplyV(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> faultUnstableSupplyV = new LeveledSignal<>(getFault_UnstableSupplyV(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> faultStatorCurrLimit = new LeveledSignal<>(getFault_StatorCurrLimit(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultSupplyCurrLimit = new LeveledSignal<>(getFault_SupplyCurrLimit(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultForwardHardLimit = new LeveledSignal<>(getFault_ForwardHardLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> faultReverseHardLimit = new LeveledSignal<>(getFault_ReverseHardLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> faultForwardSoftLimit = new LeveledSignal<>(getFault_ForwardSoftLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> faultReverseSoftLimit = new LeveledSignal<>(getFault_ReverseSoftLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> faultStaticBrakeDisabled = new LeveledSignal<>(getFault_StaticBrakeDisabled(false), TelemetryLevel.DTM, nullConsumer);
  public final LeveledSignal<Boolean> faultFusedSensorOutOfSync = new LeveledSignal<>(getFault_FusedSensorOutOfSync(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultRemoteSensorDataInvalid = new LeveledSignal<>(getFault_RemoteSensorDataInvalid(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultRemoteSensorPosOverflow = new LeveledSignal<>(getFault_RemoteSensorPosOverflow(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultRemoteSensorReset = new LeveledSignal<>(getFault_RemoteSensorReset(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultMissingHardLimitRemote = new LeveledSignal<>(getFault_MissingHardLimitRemote(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultMissingSoftLimitRemote = new LeveledSignal<>(getFault_MissingSoftLimitRemote(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultUnlicensedFeatureInUse = new LeveledSignal<>(getFault_UnlicensedFeatureInUse(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> faultUsingFusedCANcoderWhileUnlicensed = new LeveledSignal<>(getFault_UsingFusedCANcoderWhileUnlicensed(false), TelemetryLevel.DEBUG, nullConsumer);

  // --- Sticky faults ---
  public final LeveledSignal<Boolean> stickyFaultBridgeBrownout = new LeveledSignal<>(getStickyFault_BridgeBrownout(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultHardware = new LeveledSignal<>(getStickyFault_Hardware(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultBootDuringEnable = new LeveledSignal<>(getStickyFault_BootDuringEnable(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultDeviceTemp = new LeveledSignal<>(getStickyFault_DeviceTemp(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultProcTemp = new LeveledSignal<>(getStickyFault_ProcTemp(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultUndervoltage = new LeveledSignal<>(getStickyFault_Undervoltage(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultOverSupplyV = new LeveledSignal<>(getStickyFault_OverSupplyV(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultUnstableSupplyV = new LeveledSignal<>(getStickyFault_UnstableSupplyV(false), TelemetryLevel.LESS, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultStatorCurrLimit = new LeveledSignal<>(getStickyFault_StatorCurrLimit(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultSupplyCurrLimit = new LeveledSignal<>(getStickyFault_SupplyCurrLimit(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultForwardHardLimit = new LeveledSignal<>(getStickyFault_ForwardHardLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultReverseHardLimit = new LeveledSignal<>(getStickyFault_ReverseHardLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultForwardSoftLimit = new LeveledSignal<>(getStickyFault_ForwardSoftLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultReverseSoftLimit = new LeveledSignal<>(getStickyFault_ReverseSoftLimit(false), TelemetryLevel.MORE, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultStaticBrakeDisabled = new LeveledSignal<>(getStickyFault_StaticBrakeDisabled(false), TelemetryLevel.DTM, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultFusedSensorOutOfSync = new LeveledSignal<>(getStickyFault_FusedSensorOutOfSync(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultRemoteSensorDataInvalid = new LeveledSignal<>(getStickyFault_RemoteSensorDataInvalid(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultRemoteSensorPosOverflow = new LeveledSignal<>(getStickyFault_RemoteSensorPosOverflow(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultRemoteSensorReset = new LeveledSignal<>(getStickyFault_RemoteSensorReset(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultMissingDifferentialFX = new LeveledSignal<>(getStickyFault_MissingDifferentialFX(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultMissingHardLimitRemote = new LeveledSignal<>(getStickyFault_MissingHardLimitRemote(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultMissingSoftLimitRemote = new LeveledSignal<>(getStickyFault_MissingSoftLimitRemote(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultUnlicensedFeatureInUse = new LeveledSignal<>(getStickyFault_UnlicensedFeatureInUse(false), TelemetryLevel.DEBUG, nullConsumer);
  public final LeveledSignal<Boolean> stickyFaultUsingFusedCANcoderWhileUnlicensed = new LeveledSignal<>(getStickyFault_UsingFusedCANcoderWhileUnlicensed(false), TelemetryLevel.DEBUG, nullConsumer);

  private final List<LeveledSignal<?>> allSignals = new ArrayList<>();
  private final Set<LeveledSignal<?>> extraSignals = new HashSet<>();

  // Instance initializer: set loggers after all fields are initialized
  {
    position.logger = p -> Logger.recordOutput(p + "/Position", position.getValue().in(Units.Radians));
    velocity.logger = p -> Logger.recordOutput(p + "/Velocity", velocity.getValue().in(Units.RadiansPerSecond));
    acceleration.logger = p -> Logger.recordOutput(p + "/Acceleration", acceleration.getValue().in(Units.RadiansPerSecondPerSecond));
    rotorPosition.logger = p -> Logger.recordOutput(p + "/RotorPosition", rotorPosition.getValue().in(Units.Radians));
    rotorVelocity.logger = p -> Logger.recordOutput(p + "/RotorVelocity", rotorVelocity.getValue().in(Units.RadiansPerSecond));

    dutyCycle.logger = p -> Logger.recordOutput(p + "/DutyCycle", dutyCycle.getValue());
    supplyVoltage.logger = p -> Logger.recordOutput(p + "/SupplyVoltage", supplyVoltage.getValue().in(Units.Volts));
    motorVoltage.logger = p -> Logger.recordOutput(p + "/MotorVoltage", motorVoltage.getValue().in(Units.Volts));

    supplyCurrent.logger = p -> Logger.recordOutput(p + "/SupplyCurrent", supplyCurrent.getValue().in(Units.Amps));
    statorCurrent.logger = p -> Logger.recordOutput(p + "/StatorCurrent", statorCurrent.getValue().in(Units.Amps));
    torqueCurrent.logger = p -> Logger.recordOutput(p + "/TorqueCurrent", torqueCurrent.getValue().in(Units.Amps));

    deviceTemp.logger = p -> Logger.recordOutput(p + "/TempC", deviceTemp.getValue().in(Units.Celsius));
    processorTemp.logger = p -> Logger.recordOutput(p + "/ProcessorTempC", processorTemp.getValue().in(Units.Celsius));

    controlMode.logger = p -> Logger.recordOutput(p + "/ControlMode", (double) controlMode.getValue().ordinal());
    appliedRotorPolarity.logger = p -> Logger.recordOutput(p + "/AppliedRotorPolarity", appliedRotorPolarity.getValue().name());
    bridgeOutput.logger = p -> Logger.recordOutput(p + "/BridgeOutput", bridgeOutput.getValue().name());
    deviceEnable.logger = p -> Logger.recordOutput(p + "/DeviceEnable", deviceEnable.getValue().name());
    robotEnable.logger = p -> Logger.recordOutput(p + "/RobotEnable", robotEnable.getValue().name());
    motorOutputStatus.logger = p -> Logger.recordOutput(p + "/MotorOutputStatus", motorOutputStatus.getValue().name());

    closedLoopReference.logger = p -> Logger.recordOutput(p + "/ClosedLoopRef", closedLoopReference.getValue());
    closedLoopError.logger = p -> Logger.recordOutput(p + "/ClosedLoopError", closedLoopError.getValue());
    closedLoopOutput.logger = p -> Logger.recordOutput(p + "/ClosedLoopOutput", closedLoopOutput.getValue());
    closedLoopReferenceSlope.logger = p -> Logger.recordOutput(p + "/ClosedLoopRefSlope", closedLoopReferenceSlope.getValue());
    closedLoopProportionalOutput.logger = p -> Logger.recordOutput(p + "/ClosedLoopPOutput", closedLoopProportionalOutput.getValue());
    closedLoopIntegratedOutput.logger = p -> Logger.recordOutput(p + "/ClosedLoopIOutput", closedLoopIntegratedOutput.getValue());
    closedLoopDerivativeOutput.logger = p -> Logger.recordOutput(p + "/ClosedLoopDOutput", closedLoopDerivativeOutput.getValue());
    closedLoopFeedForward.logger = p -> Logger.recordOutput(p + "/ClosedLoopFF", closedLoopFeedForward.getValue());
    closedLoopSlot.logger = p -> Logger.recordOutput(p + "/ClosedLoopSlot", closedLoopSlot.getValue());

    forwardLimit.logger = p -> Logger.recordOutput(p + "/ForwardLimit", forwardLimit.getValue() == ForwardLimitValue.ClosedToGround);
    reverseLimit.logger = p -> Logger.recordOutput(p + "/ReverseLimit", reverseLimit.getValue() == ReverseLimitValue.ClosedToGround);

    motionMagicIsRunning.logger = p -> Logger.recordOutput(p + "/MotionMagicIsRunning", motionMagicIsRunning.getValue());
    motionMagicAtTarget.logger = p -> Logger.recordOutput(p + "/MotionMagicAtTarget", motionMagicAtTarget.getValue());

    // Faults
    faultBridgeBrownout.logger = p -> Logger.recordOutput(p + "/Fault/BridgeBrownout", faultBridgeBrownout.getValue());
    faultHardware.logger = p -> Logger.recordOutput(p + "/Fault/Hardware", faultHardware.getValue());
    faultBootDuringEnable.logger = p -> Logger.recordOutput(p + "/Fault/BootDuringEnable", faultBootDuringEnable.getValue());
    faultDeviceTemp.logger = p -> Logger.recordOutput(p + "/Fault/DeviceTemp", faultDeviceTemp.getValue());
    faultProcTemp.logger = p -> Logger.recordOutput(p + "/Fault/ProcTemp", faultProcTemp.getValue());
    faultUndervoltage.logger = p -> Logger.recordOutput(p + "/Fault/Undervoltage", faultUndervoltage.getValue());
    faultOverSupplyV.logger = p -> Logger.recordOutput(p + "/Fault/OverSupplyV", faultOverSupplyV.getValue());
    faultUnstableSupplyV.logger = p -> Logger.recordOutput(p + "/Fault/UnstableSupplyV", faultUnstableSupplyV.getValue());
    faultStatorCurrLimit.logger = p -> Logger.recordOutput(p + "/Fault/StatorCurrLimit", faultStatorCurrLimit.getValue());
    faultSupplyCurrLimit.logger = p -> Logger.recordOutput(p + "/Fault/SupplyCurrLimit", faultSupplyCurrLimit.getValue());
    faultForwardHardLimit.logger = p -> Logger.recordOutput(p + "/Fault/ForwardHardLimit", faultForwardHardLimit.getValue());
    faultReverseHardLimit.logger = p -> Logger.recordOutput(p + "/Fault/ReverseHardLimit", faultReverseHardLimit.getValue());
    faultForwardSoftLimit.logger = p -> Logger.recordOutput(p + "/Fault/ForwardSoftLimit", faultForwardSoftLimit.getValue());
    faultReverseSoftLimit.logger = p -> Logger.recordOutput(p + "/Fault/ReverseSoftLimit", faultReverseSoftLimit.getValue());
    faultStaticBrakeDisabled.logger = p -> Logger.recordOutput(p + "/Fault/StaticBrakeDisabled", faultStaticBrakeDisabled.getValue());
    faultFusedSensorOutOfSync.logger = p -> Logger.recordOutput(p + "/Fault/FusedSensorOutOfSync", faultFusedSensorOutOfSync.getValue());
    faultRemoteSensorDataInvalid.logger = p -> Logger.recordOutput(p + "/Fault/RemoteSensorDataInvalid", faultRemoteSensorDataInvalid.getValue());
    faultRemoteSensorPosOverflow.logger = p -> Logger.recordOutput(p + "/Fault/RemoteSensorPosOverflow", faultRemoteSensorPosOverflow.getValue());
    faultRemoteSensorReset.logger = p -> Logger.recordOutput(p + "/Fault/RemoteSensorReset", faultRemoteSensorReset.getValue());
    faultMissingHardLimitRemote.logger = p -> Logger.recordOutput(p + "/Fault/MissingHardLimitRemote", faultMissingHardLimitRemote.getValue());
    faultMissingSoftLimitRemote.logger = p -> Logger.recordOutput(p + "/Fault/MissingSoftLimitRemote", faultMissingSoftLimitRemote.getValue());
    faultUnlicensedFeatureInUse.logger = p -> Logger.recordOutput(p + "/Fault/UnlicensedFeatureInUse", faultUnlicensedFeatureInUse.getValue());
    faultUsingFusedCANcoderWhileUnlicensed.logger = p -> Logger.recordOutput(p + "/Fault/UsingFusedCANcoderWhileUnlicensed", faultUsingFusedCANcoderWhileUnlicensed.getValue());

    // Sticky faults
    stickyFaultBridgeBrownout.logger = p -> Logger.recordOutput(p + "/StickyFault/BridgeBrownout", stickyFaultBridgeBrownout.getValue());
    stickyFaultHardware.logger = p -> Logger.recordOutput(p + "/StickyFault/Hardware", stickyFaultHardware.getValue());
    stickyFaultBootDuringEnable.logger = p -> Logger.recordOutput(p + "/StickyFault/BootDuringEnable", stickyFaultBootDuringEnable.getValue());
    stickyFaultDeviceTemp.logger = p -> Logger.recordOutput(p + "/StickyFault/DeviceTemp", stickyFaultDeviceTemp.getValue());
    stickyFaultProcTemp.logger = p -> Logger.recordOutput(p + "/StickyFault/ProcTemp", stickyFaultProcTemp.getValue());
    stickyFaultUndervoltage.logger = p -> Logger.recordOutput(p + "/StickyFault/Undervoltage", stickyFaultUndervoltage.getValue());
    stickyFaultOverSupplyV.logger = p -> Logger.recordOutput(p + "/StickyFault/OverSupplyV", stickyFaultOverSupplyV.getValue());
    stickyFaultUnstableSupplyV.logger = p -> Logger.recordOutput(p + "/StickyFault/UnstableSupplyV", stickyFaultUnstableSupplyV.getValue());
    stickyFaultStatorCurrLimit.logger = p -> Logger.recordOutput(p + "/StickyFault/StatorCurrLimit", stickyFaultStatorCurrLimit.getValue());
    stickyFaultSupplyCurrLimit.logger = p -> Logger.recordOutput(p + "/StickyFault/SupplyCurrLimit", stickyFaultSupplyCurrLimit.getValue());
    stickyFaultForwardHardLimit.logger = p -> Logger.recordOutput(p + "/StickyFault/ForwardHardLimit", stickyFaultForwardHardLimit.getValue());
    stickyFaultReverseHardLimit.logger = p -> Logger.recordOutput(p + "/StickyFault/ReverseHardLimit", stickyFaultReverseHardLimit.getValue());
    stickyFaultForwardSoftLimit.logger = p -> Logger.recordOutput(p + "/StickyFault/ForwardSoftLimit", stickyFaultForwardSoftLimit.getValue());
    stickyFaultReverseSoftLimit.logger = p -> Logger.recordOutput(p + "/StickyFault/ReverseSoftLimit", stickyFaultReverseSoftLimit.getValue());
    stickyFaultStaticBrakeDisabled.logger = p -> Logger.recordOutput(p + "/StickyFault/StaticBrakeDisabled", stickyFaultStaticBrakeDisabled.getValue());
    stickyFaultFusedSensorOutOfSync.logger = p -> Logger.recordOutput(p + "/StickyFault/FusedSensorOutOfSync", stickyFaultFusedSensorOutOfSync.getValue());
    stickyFaultRemoteSensorDataInvalid.logger = p -> Logger.recordOutput(p + "/StickyFault/RemoteSensorDataInvalid", stickyFaultRemoteSensorDataInvalid.getValue());
    stickyFaultRemoteSensorPosOverflow.logger = p -> Logger.recordOutput(p + "/StickyFault/RemoteSensorPosOverflow", stickyFaultRemoteSensorPosOverflow.getValue());
    stickyFaultRemoteSensorReset.logger = p -> Logger.recordOutput(p + "/StickyFault/RemoteSensorReset", stickyFaultRemoteSensorReset.getValue());
    stickyFaultMissingDifferentialFX.logger = p -> Logger.recordOutput(p + "/StickyFault/MissingDifferentialFX", stickyFaultMissingDifferentialFX.getValue());
    stickyFaultMissingHardLimitRemote.logger = p -> Logger.recordOutput(p + "/StickyFault/MissingHardLimitRemote", stickyFaultMissingHardLimitRemote.getValue());
    stickyFaultMissingSoftLimitRemote.logger = p -> Logger.recordOutput(p + "/StickyFault/MissingSoftLimitRemote", stickyFaultMissingSoftLimitRemote.getValue());
    stickyFaultUnlicensedFeatureInUse.logger = p -> Logger.recordOutput(p + "/StickyFault/UnlicensedFeatureInUse", stickyFaultUnlicensedFeatureInUse.getValue());
    stickyFaultUsingFusedCANcoderWhileUnlicensed.logger = p -> Logger.recordOutput(p + "/StickyFault/UsingFusedCANcoderWhileUnlicensed", stickyFaultUsingFusedCANcoderWhileUnlicensed.getValue());

    // Populate allSignals list
    allSignals.addAll(List.of(
        position, velocity, acceleration, rotorPosition, rotorVelocity,
        dutyCycle, supplyVoltage, motorVoltage,
        supplyCurrent, statorCurrent, torqueCurrent,
        deviceTemp, processorTemp,
        controlMode, appliedRotorPolarity, bridgeOutput, deviceEnable, robotEnable, motorOutputStatus,
        closedLoopReference, closedLoopError, closedLoopOutput, closedLoopReferenceSlope,
        closedLoopProportionalOutput, closedLoopIntegratedOutput, closedLoopDerivativeOutput,
        closedLoopFeedForward, closedLoopSlot,
        forwardLimit, reverseLimit,
        motionMagicIsRunning, motionMagicAtTarget,
        faultBridgeBrownout, faultHardware, faultBootDuringEnable, faultDeviceTemp, faultProcTemp,
        faultUndervoltage, faultOverSupplyV, faultUnstableSupplyV, faultStatorCurrLimit, faultSupplyCurrLimit,
        faultForwardHardLimit, faultReverseHardLimit, faultForwardSoftLimit, faultReverseSoftLimit,
        faultStaticBrakeDisabled, faultFusedSensorOutOfSync, faultRemoteSensorDataInvalid,
        faultRemoteSensorPosOverflow, faultRemoteSensorReset, faultMissingHardLimitRemote,
        faultMissingSoftLimitRemote, faultUnlicensedFeatureInUse, faultUsingFusedCANcoderWhileUnlicensed,
        stickyFaultBridgeBrownout, stickyFaultHardware, stickyFaultBootDuringEnable,
        stickyFaultDeviceTemp, stickyFaultProcTemp, stickyFaultUndervoltage, stickyFaultOverSupplyV,
        stickyFaultUnstableSupplyV, stickyFaultStatorCurrLimit, stickyFaultSupplyCurrLimit,
        stickyFaultForwardHardLimit, stickyFaultReverseHardLimit, stickyFaultForwardSoftLimit,
        stickyFaultReverseSoftLimit, stickyFaultStaticBrakeDisabled, stickyFaultFusedSensorOutOfSync,
        stickyFaultRemoteSensorDataInvalid, stickyFaultRemoteSensorPosOverflow, stickyFaultRemoteSensorReset,
        stickyFaultMissingDifferentialFX, stickyFaultMissingHardLimitRemote, stickyFaultMissingSoftLimitRemote,
        stickyFaultUnlicensedFeatureInUse, stickyFaultUsingFusedCANcoderWhileUnlicensed));
  }

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

  public void addToLogging(LeveledSignal<?> signal) {
    extraSignals.add(signal);
  }

  private boolean shouldLog(LeveledSignal<?> signal) {
    return telemetryLevel.includes(signal.level) || extraSignals.contains(signal);
  }

  /**
   * Refresh cached signals from CAN based on the current telemetry level.
   * Only signals needed for the active level (or individually added via
   * addToLogging) are refreshed to conserve CAN bandwidth.
   *
   * @return Status of the CAN refresh transaction.
   */
  private StatusCode refreshSignals() {
    BaseStatusSignal[] toRefresh = allSignals.stream()
        .filter(this::shouldLog)
        .map(LeveledSignal::getBaseSignal)
        .toArray(BaseStatusSignal[]::new);
    return BaseStatusSignal.refreshAll(toRefresh);
  }

  /**
   * Publish the latest signal values to AdvantageKit (rate limited by
   * configuration). These values can be sent to network tables and WPILOG
   * simultaneously by configuring data receivers in Robot.
   */
  public void updateDashboard() {
    refreshSignals();

    for (LeveledSignal<?> signal : allSignals) {
      if (shouldLog(signal)) {
        signal.logger.accept(logPrefix);
      }
    }

    if (telemetryLevel.includes(TelemetryLevel.DEBUG)) {
      Logger.recordOutput(logPrefix + "/TorqueNm", getTorque().in(NewtonMeters));
    }
    if (telemetryLevel.includes(TelemetryLevel.DTM)) {
      Logger.recordOutput(logPrefix + "/IsProLicensed", isProLicensed);
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
