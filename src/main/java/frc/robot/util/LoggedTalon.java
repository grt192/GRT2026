package frc.robot.util;

import static edu.wpi.first.units.Units.NewtonMeters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

public class LoggedTalon extends TalonFX {
  enum TelemetryLevel {
    BASIC,
    STANDARD,
    DETAILED,
    FULL;

    boolean includes(TelemetryLevel required) {
      return this.ordinal() >= required.ordinal();
    }
  }

  private static final TelemetryLevel DEFAULT_TELEMETRY_LEVEL = TelemetryLevel.BASIC;
  private final String logPrefix;

  public TelemetryLevel dashboardTelemetryLevel = DEFAULT_TELEMETRY_LEVEL;
  public TelemetryLevel logTelemetryLevel = DEFAULT_TELEMETRY_LEVEL;

  // ===== Cached signals (refresh once, then read many) =====

  // --- Position / Velocity / Acceleration ---
  private final StatusSignal<Angle> position = getPosition(false);
  private final StatusSignal<AngularVelocity> velocity = getVelocity(false);
  private final StatusSignal<AngularAcceleration> acceleration = getAcceleration(false);
  private final StatusSignal<Angle> rotorPosition = getRotorPosition(false);
  private final StatusSignal<AngularVelocity> rotorVelocity = getRotorVelocity(false);

  // --- Duty cycle / Voltage ---
  private final StatusSignal<Double> dutyCycle = getDutyCycle(false);
  private final StatusSignal<Voltage> supplyVoltage = getSupplyVoltage(false);
  private final StatusSignal<Voltage> appliedVoltage = getMotorVoltage(false);

  // --- Current ---
  private final StatusSignal<Current> supplyCurrent = getSupplyCurrent(false);
  private final StatusSignal<Current> statorCurrent = getStatorCurrent(false);
  private final StatusSignal<Current> torqueCurrent = getTorqueCurrent(false);

  // --- Temperature ---
  private final StatusSignal<Temperature> deviceTemp = getDeviceTemp(false);
  private final StatusSignal<Temperature> processorTemp = getProcessorTemp(false);

  // --- Motor constants ---
  private final StatusSignal<Per<TorqueUnit, CurrentUnit>> motorKt = getMotorKT(false);
  private final StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> motorKv = getMotorKV(false);
  private final StatusSignal<Current> motorStallCurrent = getMotorStallCurrent(false);

  // --- Control mode / output ---
  private final StatusSignal<ControlModeValue> controlMode = getControlMode(false);
  private final StatusSignal<AppliedRotorPolarityValue> appliedRotorPolarity = getAppliedRotorPolarity(false);
  private final StatusSignal<BridgeOutputValue> bridgeOutput = getBridgeOutput(false);
  private final StatusSignal<DeviceEnableValue> deviceEnable = getDeviceEnable(false);
  private final StatusSignal<RobotEnableValue> robotEnable = getRobotEnable(false);
  private final StatusSignal<MotorOutputStatusValue> motorOutputStatus = getMotorOutputStatus(false);
  private final StatusSignal<ConnectedMotorValue> connectedMotor = getConnectedMotor(false);

  // --- Closed loop ---
  private final StatusSignal<Double> closedLoopReference = getClosedLoopReference(false);
  private final StatusSignal<Double> closedLoopError = getClosedLoopError(false);
  private final StatusSignal<Double> closedLoopOutput = getClosedLoopOutput(false);
  private final StatusSignal<Double> closedLoopReferenceSlope = getClosedLoopReferenceSlope(false);
  private final StatusSignal<Double> closedLoopProportionalOutput = getClosedLoopProportionalOutput(false);
  private final StatusSignal<Double> closedLoopIntegratedOutput = getClosedLoopIntegratedOutput(false);
  private final StatusSignal<Double> closedLoopDerivativeOutput = getClosedLoopDerivativeOutput(false);
  private final StatusSignal<Double> closedLoopFeedForward = getClosedLoopFeedForward(false);
  private final StatusSignal<Integer> closedLoopSlot = getClosedLoopSlot(false);

  // --- Limit switches ---
  private final StatusSignal<ForwardLimitValue> forwardLimit = getForwardLimit(false);
  private final StatusSignal<ReverseLimitValue> reverseLimit = getReverseLimit(false);

  // --- Motion Magic ---
  private final StatusSignal<Boolean> motionMagicIsRunning = getMotionMagicIsRunning(false);
  private final StatusSignal<Boolean> motionMagicAtTarget = getMotionMagicAtTarget(false);

  // --- Licensing ---
  private final StatusSignal<Boolean> isProLicensed = getIsProLicensed(false);

  // --- Fault fields (packed integer) ---
  private final StatusSignal<Integer> faultField = getFaultField(false);
  private final StatusSignal<Integer> stickyFaultField = getStickyFaultField(false);

  // --- Individual faults ---
  private final StatusSignal<Boolean> faultBridgeBrownout = getFault_BridgeBrownout(false);
  private final StatusSignal<Boolean> faultHardware = getFault_Hardware(false);
  private final StatusSignal<Boolean> faultBootDuringEnable = getFault_BootDuringEnable(false);
  private final StatusSignal<Boolean> faultDeviceTemp = getFault_DeviceTemp(false);
  private final StatusSignal<Boolean> faultProcTemp = getFault_ProcTemp(false);
  private final StatusSignal<Boolean> faultUndervoltage = getFault_Undervoltage(false);
  private final StatusSignal<Boolean> faultOverSupplyV = getFault_OverSupplyV(false);
  private final StatusSignal<Boolean> faultUnstableSupplyV = getFault_UnstableSupplyV(false);
  private final StatusSignal<Boolean> faultStatorCurrLimit = getFault_StatorCurrLimit(false);
  private final StatusSignal<Boolean> faultSupplyCurrLimit = getFault_SupplyCurrLimit(false);
  private final StatusSignal<Boolean> faultForwardHardLimit = getFault_ForwardHardLimit(false);
  private final StatusSignal<Boolean> faultReverseHardLimit = getFault_ReverseHardLimit(false);
  private final StatusSignal<Boolean> faultForwardSoftLimit = getFault_ForwardSoftLimit(false);
  private final StatusSignal<Boolean> faultReverseSoftLimit = getFault_ReverseSoftLimit(false);
  private final StatusSignal<Boolean> faultStaticBrakeDisabled = getFault_StaticBrakeDisabled(false);
  private final StatusSignal<Boolean> faultFusedSensorOutOfSync = getFault_FusedSensorOutOfSync(false);
  private final StatusSignal<Boolean> faultRemoteSensorDataInvalid = getFault_RemoteSensorDataInvalid(false);
  private final StatusSignal<Boolean> faultRemoteSensorPosOverflow = getFault_RemoteSensorPosOverflow(false);
  private final StatusSignal<Boolean> faultRemoteSensorReset = getFault_RemoteSensorReset(false);
  private final StatusSignal<Boolean> faultMissingDifferentialFX = getFault_MissingDifferentialFX(false);
  private final StatusSignal<Boolean> faultMissingHardLimitRemote = getFault_MissingHardLimitRemote(false);
  private final StatusSignal<Boolean> faultMissingSoftLimitRemote = getFault_MissingSoftLimitRemote(false);
  private final StatusSignal<Boolean> faultUnlicensedFeatureInUse = getFault_UnlicensedFeatureInUse(false);
  private final StatusSignal<Boolean> faultUsingFusedCANcoderWhileUnlicensed = getFault_UsingFusedCANcoderWhileUnlicensed(
      false);

  // --- Sticky faults ---
  private final StatusSignal<Boolean> stickyFaultBridgeBrownout = getStickyFault_BridgeBrownout(false);
  private final StatusSignal<Boolean> stickyFaultHardware = getStickyFault_Hardware(false);
  private final StatusSignal<Boolean> stickyFaultBootDuringEnable = getStickyFault_BootDuringEnable(false);
  private final StatusSignal<Boolean> stickyFaultDeviceTemp = getStickyFault_DeviceTemp(false);
  private final StatusSignal<Boolean> stickyFaultProcTemp = getStickyFault_ProcTemp(false);
  private final StatusSignal<Boolean> stickyFaultUndervoltage = getStickyFault_Undervoltage(false);
  private final StatusSignal<Boolean> stickyFaultOverSupplyV = getStickyFault_OverSupplyV(false);
  private final StatusSignal<Boolean> stickyFaultUnstableSupplyV = getStickyFault_UnstableSupplyV(false);
  private final StatusSignal<Boolean> stickyFaultStatorCurrLimit = getStickyFault_StatorCurrLimit(false);
  private final StatusSignal<Boolean> stickyFaultSupplyCurrLimit = getStickyFault_SupplyCurrLimit(false);
  private final StatusSignal<Boolean> stickyFaultForwardHardLimit = getStickyFault_ForwardHardLimit(false);
  private final StatusSignal<Boolean> stickyFaultReverseHardLimit = getStickyFault_ReverseHardLimit(false);
  private final StatusSignal<Boolean> stickyFaultForwardSoftLimit = getStickyFault_ForwardSoftLimit(false);
  private final StatusSignal<Boolean> stickyFaultReverseSoftLimit = getStickyFault_ReverseSoftLimit(false);
  private final StatusSignal<Boolean> stickyFaultStaticBrakeDisabled = getStickyFault_StaticBrakeDisabled(false);
  private final StatusSignal<Boolean> stickyFaultFusedSensorOutOfSync = getStickyFault_FusedSensorOutOfSync(
      false);
  private final StatusSignal<Boolean> stickyFaultRemoteSensorDataInvalid = getStickyFault_RemoteSensorDataInvalid(
      false);
  private final StatusSignal<Boolean> stickyFaultRemoteSensorPosOverflow = getStickyFault_RemoteSensorPosOverflow(
      false);
  private final StatusSignal<Boolean> stickyFaultRemoteSensorReset = getStickyFault_RemoteSensorReset(false);
  private final StatusSignal<Boolean> stickyFaultMissingDifferentialFX = getStickyFault_MissingDifferentialFX(
      false);
  private final StatusSignal<Boolean> stickyFaultMissingHardLimitRemote = getStickyFault_MissingHardLimitRemote(
      false);
  private final StatusSignal<Boolean> stickyFaultMissingSoftLimitRemote = getStickyFault_MissingSoftLimitRemote(
      false);
  private final StatusSignal<Boolean> stickyFaultUnlicensedFeatureInUse = getStickyFault_UnlicensedFeatureInUse(
      false);
  private final StatusSignal<Boolean> stickyFaultUsingFusedCANcoderWhileUnlicensed = getStickyFault_UsingFusedCANcoderWhileUnlicensed(
      false);

  private Per<TorqueUnit, CurrentUnit> kt;
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

  /**
   * Refresh cached signals from CAN based on the current telemetry level.
   * Only signals needed for the active level are refreshed to conserve
   * CAN bandwidth.
   *
   * @return Status of the CAN refresh transaction.
   */
  private StatusCode refreshSignals() {
    TelemetryLevel level = getEffectiveTelemetryLevel();

    // Always refresh BASIC signals
    StatusCode status = BaseStatusSignal.refreshAll(
        position,
        velocity,
        dutyCycle,
        appliedVoltage,
        controlMode,
        forwardLimit,
        reverseLimit,
        faultBridgeBrownout,
        faultHardware,
        faultBootDuringEnable);

    if (!level.includes(TelemetryLevel.STANDARD))
      return status;
    BaseStatusSignal.refreshAll(
        acceleration,
        supplyVoltage,
        supplyCurrent,
        rotorPosition,
        rotorVelocity,
        deviceEnable,
        faultDeviceTemp,
        faultUndervoltage,
        faultOverSupplyV,
        faultStatorCurrLimit,
        faultSupplyCurrLimit);

    if (!level.includes(TelemetryLevel.DETAILED))
      return status;
    BaseStatusSignal.refreshAll(
        statorCurrent,
        deviceTemp,
        processorTemp,
        appliedRotorPolarity,
        bridgeOutput,
        motorOutputStatus,
        motionMagicIsRunning,
        motionMagicAtTarget,
        closedLoopOutput,
        closedLoopSlot,
        faultUnstableSupplyV,
        faultForwardHardLimit,
        faultReverseHardLimit,
        faultForwardSoftLimit,
        faultReverseSoftLimit,
        faultProcTemp,
        faultStaticBrakeDisabled);

    if (!level.includes(TelemetryLevel.FULL))
      return status;
    BaseStatusSignal.refreshAll(
        torqueCurrent,
        motorKt,
        motorKv,
        motorStallCurrent,
        closedLoopReference,
        closedLoopError,
        closedLoopReferenceSlope,
        closedLoopProportionalOutput,
        closedLoopIntegratedOutput,
        closedLoopDerivativeOutput,
        closedLoopFeedForward,
        robotEnable,
        connectedMotor,
        isProLicensed,
        faultField,
        stickyFaultField,
        faultFusedSensorOutOfSync,
        faultRemoteSensorDataInvalid,
        faultRemoteSensorPosOverflow,
        faultRemoteSensorReset,
        faultMissingDifferentialFX,
        faultMissingHardLimitRemote,
        faultMissingSoftLimitRemote,
        faultUnlicensedFeatureInUse,
        faultUsingFusedCANcoderWhileUnlicensed,
        stickyFaultBridgeBrownout,
        stickyFaultHardware,
        stickyFaultBootDuringEnable,
        stickyFaultDeviceTemp,
        stickyFaultProcTemp,
        stickyFaultUndervoltage,
        stickyFaultOverSupplyV,
        stickyFaultUnstableSupplyV,
        stickyFaultStatorCurrLimit,
        stickyFaultSupplyCurrLimit,
        stickyFaultForwardHardLimit,
        stickyFaultReverseHardLimit,
        stickyFaultForwardSoftLimit,
        stickyFaultReverseSoftLimit,
        stickyFaultStaticBrakeDisabled,
        stickyFaultFusedSensorOutOfSync,
        stickyFaultRemoteSensorDataInvalid,
        stickyFaultRemoteSensorPosOverflow,
        stickyFaultRemoteSensorReset,
        stickyFaultMissingDifferentialFX,
        stickyFaultMissingHardLimitRemote,
        stickyFaultMissingSoftLimitRemote,
        stickyFaultUnlicensedFeatureInUse,
        stickyFaultUsingFusedCANcoderWhileUnlicensed);

    return status;
  }

  /**
   * Publish the latest signal values to AdvantageKit (rate limited by
   * configuration). These values can be sent to network tables and WPILOG
   * simultaneously by configuring data receivers in Robot.
   */
  public void updateDashboard() {
    refreshSignals();

    TelemetryLevel telemetryLevel = getEffectiveTelemetryLevel();

    if (telemetryLevel.includes(TelemetryLevel.BASIC)) {
      Logger.recordOutput(logPrefix + "/Position", position.getValue().in(Units.Radians));
      Logger.recordOutput(logPrefix + "/Velocity", velocity.getValue().in(Units.RadiansPerSecond));
      Logger.recordOutput(logPrefix + "/DutyCycle", dutyCycle.getValue());
      Logger.recordOutput(logPrefix + "/AppliedVoltage", appliedVoltage.getValue().in(Units.Volts));
      Logger.recordOutput(logPrefix + "/ControlMode", (double) controlMode.getValue().ordinal());

      Logger.recordOutput(logPrefix + "/ForwardLimit", isForwardLimitClosed());
      Logger.recordOutput(logPrefix + "/ReverseLimit", isReverseLimitClosed());

      Logger.recordOutput(logPrefix + "/Fault/Brownout", faultBridgeBrownout.getValue());
      Logger.recordOutput(logPrefix + "/Fault/Hardware", faultHardware.getValue());
      Logger.recordOutput(logPrefix + "/Fault/BootDuringEnable", faultBootDuringEnable.getValue());
    }

    if (telemetryLevel.includes(TelemetryLevel.STANDARD)) {
      Logger.recordOutput(
          logPrefix + "/Acceleration",
          acceleration.getValue().in(Units.RadiansPerSecondPerSecond));
      Logger.recordOutput(logPrefix + "/SupplyVoltage", supplyVoltage.getValue().in(Units.Volts));
      Logger.recordOutput(logPrefix + "/SupplyCurrent", supplyCurrent.getValue().in(Units.Amps));

      Logger.recordOutput(logPrefix + "/RotorPosition", rotorPosition.getValue().in(Units.Radians));
      Logger.recordOutput(logPrefix + "/RotorVelocity",
          rotorVelocity.getValue().in(Units.RadiansPerSecond));
      Logger.recordOutput(logPrefix + "/DeviceEnable", deviceEnable.getValue().name());

      Logger.recordOutput(logPrefix + "/Fault/DeviceTemp", faultDeviceTemp.getValue());
      Logger.recordOutput(logPrefix + "/Fault/Undervoltage", faultUndervoltage.getValue());
      Logger.recordOutput(logPrefix + "/Fault/OverSupplyV", faultOverSupplyV.getValue());
      Logger.recordOutput(logPrefix + "/Fault/StatorCurrLimit", faultStatorCurrLimit.getValue());
      Logger.recordOutput(logPrefix + "/Fault/SupplyCurrLimit", faultSupplyCurrLimit.getValue());
    }

    if (telemetryLevel.includes(TelemetryLevel.DETAILED)) {
      Logger.recordOutput(logPrefix + "/StatorCurrent", statorCurrent.getValue().in(Units.Amps));
      Logger.recordOutput(logPrefix + "/TempC", deviceTemp.getValue().in(Units.Celsius));
      Logger.recordOutput(logPrefix + "/ProcessorTempC", processorTemp.getValue().in(Units.Celsius));

      Logger.recordOutput(logPrefix + "/AppliedRotorPolarity",
          appliedRotorPolarity.getValue().name());
      Logger.recordOutput(logPrefix + "/BridgeOutput", bridgeOutput.getValue().name());
      Logger.recordOutput(logPrefix + "/MotorOutputStatus", motorOutputStatus.getValue().name());

      Logger.recordOutput(logPrefix + "/MotionMagicIsRunning", motionMagicIsRunning.getValue());
      Logger.recordOutput(logPrefix + "/MotionMagicAtTarget", motionMagicAtTarget.getValue());

      Logger.recordOutput(logPrefix + "/ClosedLoopOutput", closedLoopOutput.getValue());
      Logger.recordOutput(logPrefix + "/ClosedLoopSlot", closedLoopSlot.getValue());

      Logger.recordOutput(logPrefix + "/Fault/UnstableSupplyV", faultUnstableSupplyV.getValue());
      Logger.recordOutput(logPrefix + "/Fault/ForwardHardLimit", faultForwardHardLimit.getValue());
      Logger.recordOutput(logPrefix + "/Fault/ReverseHardLimit", faultReverseHardLimit.getValue());
      Logger.recordOutput(logPrefix + "/Fault/ForwardSoftLimit", faultForwardSoftLimit.getValue());
      Logger.recordOutput(logPrefix + "/Fault/ReverseSoftLimit", faultReverseSoftLimit.getValue());
      Logger.recordOutput(logPrefix + "/Fault/ProcTemp", faultProcTemp.getValue());
      Logger.recordOutput(logPrefix + "/Fault/StaticBrakeDisabled",
          faultStaticBrakeDisabled.getValue());
    }

    if (telemetryLevel.includes(TelemetryLevel.FULL)) {
      Logger.recordOutput(logPrefix + "/TorqueCurrent", torqueCurrent.getValue().in(Units.Amps));
      Logger.recordOutput(logPrefix + "/TorqueNm", getTorque().in(NewtonMeters));
      Logger.recordOutput(logPrefix + "/ClosedLoopRef", closedLoopReference.getValue());
      Logger.recordOutput(logPrefix + "/ClosedLoopError", closedLoopError.getValue());
      Logger.recordOutput(logPrefix + "/ClosedLoopRefSlope", closedLoopReferenceSlope.getValue());
      Logger.recordOutput(logPrefix + "/ClosedLoopPOutput", closedLoopProportionalOutput.getValue());
      Logger.recordOutput(logPrefix + "/ClosedLoopIOutput", closedLoopIntegratedOutput.getValue());
      Logger.recordOutput(logPrefix + "/ClosedLoopDOutput", closedLoopDerivativeOutput.getValue());
      Logger.recordOutput(logPrefix + "/ClosedLoopFF", closedLoopFeedForward.getValue());

      Logger.recordOutput(logPrefix + "/MotorStallCurrent",
          motorStallCurrent.getValue().in(Units.Amps));
      Logger.recordOutput(logPrefix + "/MotorKt", motorKt.getValue().baseUnitMagnitude());
      Logger.recordOutput(logPrefix + "/MotorKv", motorKv.getValue().baseUnitMagnitude());

      Logger.recordOutput(logPrefix + "/RobotEnable", robotEnable.getValue().name());
      Logger.recordOutput(logPrefix + "/ConnectedMotor", connectedMotor.getValue().name());
      Logger.recordOutput(logPrefix + "/IsProLicensed", isProLicensed.getValue());

      Logger.recordOutput(logPrefix + "/FaultField", faultField.getValue());
      Logger.recordOutput(logPrefix + "/StickyFaultField", stickyFaultField.getValue());

      Logger.recordOutput(logPrefix + "/Fault/FusedSensorOutOfSync",
          faultFusedSensorOutOfSync.getValue());
      Logger.recordOutput(logPrefix + "/Fault/RemoteSensorDataInvalid",
          faultRemoteSensorDataInvalid.getValue());
      Logger.recordOutput(logPrefix + "/Fault/RemoteSensorPosOverflow",
          faultRemoteSensorPosOverflow.getValue());
      Logger.recordOutput(logPrefix + "/Fault/RemoteSensorReset", faultRemoteSensorReset.getValue());
      Logger.recordOutput(logPrefix + "/Fault/MissingDifferentialFX",
          faultMissingDifferentialFX.getValue());
      Logger.recordOutput(logPrefix + "/Fault/MissingHardLimitRemote",
          faultMissingHardLimitRemote.getValue());
      Logger.recordOutput(logPrefix + "/Fault/MissingSoftLimitRemote",
          faultMissingSoftLimitRemote.getValue());
      Logger.recordOutput(logPrefix + "/Fault/UnlicensedFeatureInUse",
          faultUnlicensedFeatureInUse.getValue());
      Logger.recordOutput(logPrefix + "/Fault/UsingFusedCANcoderWhileUnlicensed",
          faultUsingFusedCANcoderWhileUnlicensed.getValue());

      // Sticky faults
      Logger.recordOutput(logPrefix + "/StickyFault/Brownout", stickyFaultBridgeBrownout.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/Hardware", stickyFaultHardware.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/BootDuringEnable",
          stickyFaultBootDuringEnable.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/DeviceTemp", stickyFaultDeviceTemp.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/ProcTemp", stickyFaultProcTemp.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/Undervoltage",
          stickyFaultUndervoltage.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/OverSupplyV", stickyFaultOverSupplyV.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/UnstableSupplyV",
          stickyFaultUnstableSupplyV.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/StatorCurrLimit",
          stickyFaultStatorCurrLimit.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/SupplyCurrLimit",
          stickyFaultSupplyCurrLimit.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/ForwardHardLimit",
          stickyFaultForwardHardLimit.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/ReverseHardLimit",
          stickyFaultReverseHardLimit.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/ForwardSoftLimit",
          stickyFaultForwardSoftLimit.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/ReverseSoftLimit",
          stickyFaultReverseSoftLimit.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/StaticBrakeDisabled",
          stickyFaultStaticBrakeDisabled.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/FusedSensorOutOfSync",
          stickyFaultFusedSensorOutOfSync.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/RemoteSensorDataInvalid",
          stickyFaultRemoteSensorDataInvalid.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/RemoteSensorPosOverflow",
          stickyFaultRemoteSensorPosOverflow.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/RemoteSensorReset",
          stickyFaultRemoteSensorReset.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/MissingDifferentialFX",
          stickyFaultMissingDifferentialFX.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/MissingHardLimitRemote",
          stickyFaultMissingHardLimitRemote.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/MissingSoftLimitRemote",
          stickyFaultMissingSoftLimitRemote.getValue());
      Logger.recordOutput(
          logPrefix + "/StickyFault/UnlicensedFeatureInUse",
          stickyFaultUnlicensedFeatureInUse.getValue());
      Logger.recordOutput(logPrefix + "/StickyFault/UsingFusedCANcoderWhileUnlicensed",
          stickyFaultUsingFusedCANcoderWhileUnlicensed.getValue());
    }
  }

  private TelemetryLevel getEffectiveTelemetryLevel() {
    return dashboardTelemetryLevel.ordinal() >= logTelemetryLevel.ordinal()
        ? dashboardTelemetryLevel
        : logTelemetryLevel;
  }

  private boolean isForwardLimitClosed() {
    return forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
  }

  private boolean isReverseLimitClosed() {
    return reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
  }

  // torque = kt * torqueCurrent
  private Torque getTorque() {
    // kt = torque constant
    Current tCurrent = torqueCurrent.getValue();
    return (Torque) kt.timesDivisor(tCurrent);
  }
}
