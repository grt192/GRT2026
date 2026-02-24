package frc.robot.util;

import java.util.EnumSet;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.DebugConstants.MASTER_DEBUG;

/**
 * A wrapper for TalonFX that provides comprehensive logging and telemetry.
 *
 * <h2>Usage Example:</h2>
 * <pre>{@code
 * // Create configuration
 * TalonFXConfiguration config = new TalonFXConfiguration();
 * config.Slot0.kP = 1.0;
 * config.Slot0.kI = 0.0;
 * config.Slot0.kD = 0.05;
 * config.Slot0.kV = 0.12;
 *
 * // Create LoggedTalon with a descriptive name
 * LoggedTalon armMotor = new LoggedTalon("Arm", 5, "canivore", config);
 *
 * // Or with FOC enabled for smoother control
 * LoggedTalon shooterMotor = new LoggedTalon("Shooter", 6, "canivore", config, true);
 * }</pre>
 *
 * <h2>In Subsystem Periodic:</h2>
 * <pre>{@code
 * @Override
 * public void periodic() {
 *     // Refresh all signals once per loop (efficient batch read)
 *     motor.refreshSignals();
 *
 *     // Publish to NetworkTables for SmartDashboard/Shuffleboard
 *     motor.publishStats();
 *
 *     // Log to file for post-match analysis
 *     motor.logStats();
 * }
 * }</pre>
 *
 * <h2>Control Modes:</h2>
 * <pre>{@code
 * // Open Loop
 * motor.setDutyCycle(0.5);           // -1.0 to 1.0
 * motor.setVoltage(6.0);             // Direct voltage
 * motor.setTorqueCurrent(10.0);      // Amps (FOC)
 *
 * // Position Control (rotations)
 * motor.setPositionReference(5.0);              // Voltage-based
 * motor.setPositionReference(5.0, 0.5);         // With feedforward
 * motor.setPositionTorqueFOC(5.0);              // FOC-based
 *
 * // Velocity Control (rotations per second)
 * motor.setVelocityReference(50.0);             // Voltage-based
 * motor.setVelocityReference(50.0, 1.0);        // With feedforward
 * motor.setVelocityTorqueFOC(50.0);             // FOC-based
 *
 * // Motion Magic (profiled motion)
 * motor.setMotionMagicPosition(10.0);           // Trapezoidal profile
 * motor.setMotionMagicExpoPosition(10.0);       // S-curve profile
 * motor.setMotionMagicVelocity(100.0);          // Velocity with accel limit
 * }</pre>
 *
 * <h2>Reading Values:</h2>
 * <pre>{@code
 * double pos = motor.getPosition();           // Rotations
 * double vel = motor.getVelocity();           // RPS
 * double accel = motor.getAcceleration();     // RPS²
 * double error = motor.getClosedLoopError();
 * double temp = motor.getTemperature();       // Celsius
 * double current = motor.getStatorCurrent();  // Amps
 * }</pre>
 *
 * <h2>NetworkTables Structure:</h2>
 * Data appears under {@code MotorStats/<name>/}:
 * <ul>
 *   <li>State: position, velocity, acceleration, appliedVolts, dutyCycle</li>
 *   <li>Current: supplyCurrent, statorCurrent, torqueCurrent</li>
 *   <li>Targets: targetPosition, targetVelocity, targetVoltage, targetDutyCycle</li>
 *   <li>Closed Loop: closedLoop/error, closedLoop/pOutput, closedLoop/iOutput,
 *       closedLoop/dOutput, closedLoop/ffOutput, closedLoop/reference, closedLoop/output</li>
 *   <li>Faults: faults/hardware, faults/deviceTemp, faults/undervoltage, etc.</li>
 * </ul>
 */
public class LoggedTalon {

    private final TalonFX motor;
    private final int canId;
    private final String name;
    private double[] pidsvg = new double[6];
    private boolean useFOC = false;

    // Cached status signals for efficiency
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Double> dutyCycleSignal;
    private final StatusSignal<Double> closedLoopErrorSignal;
    private final StatusSignal<Double> closedLoopOutputSignal;
    private final StatusSignal<Double> closedLoopReferenceSignal;
    private final StatusSignal<Double> closedLoopPOutputSignal;
    private final StatusSignal<Double> closedLoopIOutputSignal;
    private final StatusSignal<Double> closedLoopDOutputSignal;
    private final StatusSignal<Double> closedLoopFeedForwardSignal;

    // Reusable control requests
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);

    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);

    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

    private final MotionMagicDutyCycle motionMagicDutyCycle = new MotionMagicDutyCycle(0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);
    private final MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

    private NetworkTableInstance ntInstance;
    private NetworkTable motorStatsTable;

    // State publishers
    private DoublePublisher positionPublisher;
    private DoublePublisher velocityPublisher;
    private DoublePublisher accelerationPublisher;
    private DoublePublisher appliedVoltsPublisher;
    private DoublePublisher supplyCurrentPublisher;
    private DoublePublisher statorCurrentPublisher;
    private DoublePublisher torqueCurrentPublisher;
    private DoublePublisher temperaturePublisher;
    private DoublePublisher dutyCyclePublisher;

    // Target publishers
    private DoublePublisher targetPositionPublisher;
    private DoublePublisher targetVelocityPublisher;
    private DoublePublisher targetVoltagePublisher;
    private DoublePublisher targetDutyCyclePublisher;
    private DoublePublisher targetTorqueCurrentPublisher;

    // Closed loop publishers
    private DoublePublisher closedLoopErrorPublisher;
    private DoublePublisher closedLoopOutputPublisher;
    private DoublePublisher closedLoopReferencePublisher;
    private DoublePublisher closedLoopPOutputPublisher;
    private DoublePublisher closedLoopIOutputPublisher;
    private DoublePublisher closedLoopDOutputPublisher;
    private DoublePublisher closedLoopFFOutputPublisher;

    // Fault publishers
    private BooleanPublisher faultHardwarePublisher;
    private BooleanPublisher faultBootDuringEnablePublisher;
    private BooleanPublisher faultDeviceTempPublisher;
    private BooleanPublisher faultProcTempPublisher;
    private BooleanPublisher faultUndervoltagePublisher;
    private BooleanPublisher faultSupplyCurrLimitPublisher;
    private BooleanPublisher faultStatorCurrLimitPublisher;

    // State log entries
    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry velocityLogEntry;
    private DoubleLogEntry accelerationLogEntry;
    private DoubleLogEntry appliedVoltsLogEntry;
    private DoubleLogEntry supplyCurrentLogEntry;
    private DoubleLogEntry statorCurrentLogEntry;
    private DoubleLogEntry torqueCurrentLogEntry;
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry dutyCycleLogEntry;

    // Target log entries
    private DoubleLogEntry targetPositionLogEntry;
    private DoubleLogEntry targetVelocityLogEntry;
    private DoubleLogEntry targetVoltageLogEntry;
    private DoubleLogEntry targetDutyCycleLogEntry;
    private DoubleLogEntry targetTorqueCurrentLogEntry;

    // Closed loop log entries
    private DoubleLogEntry closedLoopErrorLogEntry;
    private DoubleLogEntry closedLoopOutputLogEntry;
    private DoubleLogEntry closedLoopReferenceLogEntry;
    private DoubleLogEntry closedLoopPOutputLogEntry;
    private DoubleLogEntry closedLoopIOutputLogEntry;
    private DoubleLogEntry closedLoopDOutputLogEntry;
    private DoubleLogEntry closedLoopFFOutputLogEntry;

    // Fault log entries
    private BooleanLogEntry faultHardwareLogEntry;
    private BooleanLogEntry faultBootDuringEnableLogEntry;
    private BooleanLogEntry faultDeviceTempLogEntry;
    private BooleanLogEntry faultProcTempLogEntry;
    private BooleanLogEntry faultUndervoltageLogEntry;
    private BooleanLogEntry faultSupplyCurrLimitLogEntry;
    private BooleanLogEntry faultStatorCurrLimitLogEntry;

    // Target values
    private double targetPosition;
    private double targetVelocity;
    private double targetVoltage;
    private double targetDutyCycle;
    private double targetTorqueCurrent;

    public LoggedTalon(String name, int canId, String canBusName, TalonFXConfiguration talonConfig) {
        this(name, canId, canBusName, talonConfig, false);
    }

    public LoggedTalon(String name, int canId, String canBusName, TalonFXConfiguration talonConfig, boolean useFOC) {
        motor = new TalonFX(canId, canBusName);
        for (int i = 0; i < 4; i++) {
            boolean error = motor.getConfigurator()
                .apply(talonConfig, 0.1) == StatusCode.OK;
            if (!error) break;
        }
        this.name = name;
        this.canId = canId;
        this.useFOC = useFOC;
        pidsvg = new double[] {
            talonConfig.Slot0.kP,
            talonConfig.Slot0.kI,
            talonConfig.Slot0.kD,
            talonConfig.Slot0.kS,
            talonConfig.Slot0.kV,
            talonConfig.Slot0.kG
        };

        // Cache status signals for efficiency
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();
        motorVoltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        torqueCurrentSignal = motor.getTorqueCurrent();
        temperatureSignal = motor.getDeviceTemp();
        dutyCycleSignal = motor.getDutyCycle();
        closedLoopErrorSignal = motor.getClosedLoopError();
        closedLoopOutputSignal = motor.getClosedLoopOutput();
        closedLoopReferenceSignal = motor.getClosedLoopReference();
        closedLoopPOutputSignal = motor.getClosedLoopProportionalOutput();
        closedLoopIOutputSignal = motor.getClosedLoopIntegratedOutput();
        closedLoopDOutputSignal = motor.getClosedLoopDerivativeOutput();
        closedLoopFeedForwardSignal = motor.getClosedLoopFeedForward();

        initNT(name);
        initLogs(name);
        if (MASTER_DEBUG) {
            enableDebug();
        }
    }

    /**
     * Enables or disables FOC (Field Oriented Control) for applicable control modes.
     * FOC provides smoother control and better efficiency but requires more processing.
     * @param enable true to enable FOC, false to disable
     */
    public void setFOCEnabled(boolean enable) {
        this.useFOC = enable;
        // Update FOC setting on voltage-based controls
        dutyCycleOut.EnableFOC = enable;
        voltageOut.EnableFOC = enable;
        positionDutyCycle.EnableFOC = enable;
        positionVoltage.EnableFOC = enable;
        velocityDutyCycle.EnableFOC = enable;
        velocityVoltage.EnableFOC = enable;
        motionMagicDutyCycle.EnableFOC = enable;
        motionMagicVoltage.EnableFOC = enable;
        motionMagicExpoVoltage.EnableFOC = enable;
        motionMagicVelocityVoltage.EnableFOC = enable;
    }

    /**
     * @return true if FOC is enabled
     */
    public boolean isFOCEnabled() {
        return useFOC;
    }

    // ==================== OPEN LOOP CONTROL ====================

    /**
     * Set duty cycle output (-1.0 to 1.0)
     * @param output duty cycle output
     */
    public void setDutyCycle(double output) {
        targetDutyCycle = output;
        motor.setControl(dutyCycleOut.withOutput(output));
    }

    /**
     * Set voltage output directly
     * @param voltage target voltage
     */
    public void setVoltage(double voltage) {
        targetVoltage = voltage;
        motor.setControl(voltageOut.withOutput(voltage));
    }

    /**
     * Set torque current output (FOC mode)
     * @param current target current in amps
     */
    public void setTorqueCurrent(double current) {
        targetTorqueCurrent = current;
        motor.setControl(torqueCurrentFOC.withOutput(current));
    }

    /**
     * Simple power control using motor.set()
     * @param power power from -1.0 to 1.0
     */
    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * Alias for setPower
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    // ==================== POSITION CONTROL ====================

    /**
     * Set position reference using voltage control
     * @param position target position in rotations
     */
    public void setPositionReference(double position) {
        targetPosition = position;
        motor.setControl(positionVoltage.withPosition(position));
    }

    /**
     * Set position reference with feedforward
     * @param position target position in rotations
     * @param feedforward feedforward voltage
     */
    public void setPositionReference(double position, double feedforward) {
        targetPosition = position;
        motor.setControl(positionVoltage.withPosition(position).withFeedForward(feedforward));
    }

    /**
     * Set position reference using duty cycle control
     * @param position target position in rotations
     */
    public void setPositionDutyCycle(double position) {
        targetPosition = position;
        motor.setControl(positionDutyCycle.withPosition(position));
    }

    /**
     * Set position reference using duty cycle control with feedforward
     * @param position target position in rotations
     * @param feedforward feedforward duty cycle
     */
    public void setPositionDutyCycle(double position, double feedforward) {
        targetPosition = position;
        motor.setControl(positionDutyCycle.withPosition(position).withFeedForward(feedforward));
    }

    /**
     * Set position reference using torque current FOC
     * @param position target position in rotations
     */
    public void setPositionTorqueFOC(double position) {
        targetPosition = position;
        motor.setControl(positionTorqueCurrentFOC.withPosition(position));
    }

    /**
     * Set position reference using torque current FOC with feedforward
     * @param position target position in rotations
     * @param feedforward feedforward current in amps
     */
    public void setPositionTorqueFOC(double position, double feedforward) {
        targetPosition = position;
        motor.setControl(positionTorqueCurrentFOC.withPosition(position).withFeedForward(feedforward));
    }

    // ==================== VELOCITY CONTROL ====================

    /**
     * Set velocity reference using voltage control
     * @param velocity target velocity in rotations per second
     */
    public void setVelocityReference(double velocity) {
        targetVelocity = velocity;
        motor.setControl(velocityVoltage.withVelocity(velocity));
    }

    /**
     * Set velocity reference with feedforward
     * @param velocity target velocity in rotations per second
     * @param feedforward feedforward voltage
     */
    public void setVelocityReference(double velocity, double feedforward) {
        targetVelocity = velocity;
        motor.setControl(velocityVoltage.withVelocity(velocity).withFeedForward(feedforward));
    }

    /**
     * Set velocity reference using duty cycle control
     * @param velocity target velocity in rotations per second
     */
    public void setVelocityDutyCycle(double velocity) {
        targetVelocity = velocity;
        motor.setControl(velocityDutyCycle.withVelocity(velocity));
    }

    /**
     * Set velocity reference using duty cycle control with feedforward
     * @param velocity target velocity in rotations per second
     * @param feedforward feedforward duty cycle
     */
    public void setVelocityDutyCycle(double velocity, double feedforward) {
        targetVelocity = velocity;
        motor.setControl(velocityDutyCycle.withVelocity(velocity).withFeedForward(feedforward));
    }

    /**
     * Set velocity reference using torque current FOC
     * @param velocity target velocity in rotations per second
     */
    public void setVelocityTorqueFOC(double velocity) {
        targetVelocity = velocity;
        motor.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
    }

    /**
     * Set velocity reference using torque current FOC with feedforward
     * @param velocity target velocity in rotations per second
     * @param feedforward feedforward current in amps
     */
    public void setVelocityTorqueFOC(double velocity, double feedforward) {
        targetVelocity = velocity;
        motor.setControl(velocityTorqueCurrentFOC.withVelocity(velocity).withFeedForward(feedforward));
    }

    // ==================== MOTION MAGIC POSITION ====================

    /**
     * Set motion magic position using voltage control
     * @param position target position in rotations
     */
    public void setMotionMagicPosition(double position) {
        targetPosition = position;
        motor.setControl(motionMagicVoltage.withPosition(position));
    }

    /**
     * Set motion magic position with feedforward
     * @param position target position in rotations
     * @param feedforward feedforward voltage
     */
    public void setMotionMagicPosition(double position, double feedforward) {
        targetPosition = position;
        motor.setControl(motionMagicVoltage.withPosition(position).withFeedForward(feedforward));
    }

    /**
     * Set motion magic position using duty cycle
     * @param position target position in rotations
     */
    public void setMotionMagicPositionDutyCycle(double position) {
        targetPosition = position;
        motor.setControl(motionMagicDutyCycle.withPosition(position));
    }

    /**
     * Set motion magic position using torque current FOC
     * @param position target position in rotations
     */
    public void setMotionMagicPositionTorqueFOC(double position) {
        targetPosition = position;
        motor.setControl(motionMagicTorqueCurrentFOC.withPosition(position));
    }

    /**
     * Set motion magic position using torque current FOC with feedforward
     * @param position target position in rotations
     * @param feedforward feedforward current in amps
     */
    public void setMotionMagicPositionTorqueFOC(double position, double feedforward) {
        targetPosition = position;
        motor.setControl(motionMagicTorqueCurrentFOC.withPosition(position).withFeedForward(feedforward));
    }

    /**
     * Set motion magic expo position (S-curve profile)
     * @param position target position in rotations
     */
    public void setMotionMagicExpoPosition(double position) {
        targetPosition = position;
        motor.setControl(motionMagicExpoVoltage.withPosition(position));
    }

    /**
     * Set motion magic expo position with feedforward
     * @param position target position in rotations
     * @param feedforward feedforward voltage
     */
    public void setMotionMagicExpoPosition(double position, double feedforward) {
        targetPosition = position;
        motor.setControl(motionMagicExpoVoltage.withPosition(position).withFeedForward(feedforward));
    }

    // ==================== MOTION MAGIC VELOCITY ====================

    /**
     * Set motion magic velocity
     * @param velocity target velocity in rotations per second
     */
    public void setMotionMagicVelocity(double velocity) {
        targetVelocity = velocity;
        motor.setControl(motionMagicVelocityVoltage.withVelocity(velocity));
    }

    /**
     * Set motion magic velocity with feedforward and acceleration
     * @param velocity target velocity in rotations per second
     * @param feedforward feedforward voltage
     * @param acceleration target acceleration
     */
    public void setMotionMagicVelocity(double velocity, double feedforward, double acceleration) {
        targetVelocity = velocity;
        motor.setControl(motionMagicVelocityVoltage
            .withVelocity(velocity)
            .withFeedForward(feedforward)
            .withAcceleration(acceleration));
    }

    // ==================== ENCODER/POSITION UTILITIES ====================

    /**
     * Sets the motor's internal position
     * @param position position value in rotations
     */
    public void setPosition(double position) {
        for (int i = 0; i < 4; i++) {
            boolean error = motor.setPosition(position) == StatusCode.OK;
            if (!error) break;
        }
    }

    /**
     * Resets the encoder position to zero
     */
    public void resetEncoder() {
        motor.setPosition(0.0);
    }

    // ==================== CONFIGURATION ====================

    /**
     * Configures PIDSVG gains for slot 0
     * @param p kP
     * @param i kI
     * @param d kD
     * @param s kS for static friction
     * @param v kV velocity feedforward
     * @param g kG gravity compensation
     */
    public void configurePIDSVG(double p, double i, double d, double s, double v, double g) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;
        slot0Configs.kV = v;
        slot0Configs.kG = g;
        motor.getConfigurator().apply(slot0Configs);
    }

    // ==================== NETWORK TABLES ====================

    private void initNT(String name) {
        ntInstance = NetworkTableInstance.getDefault();
        motorStatsTable = ntInstance.getTable("MotorStats/" + name);

        // State publishers
        positionPublisher = motorStatsTable.getDoubleTopic("position").publish();
        velocityPublisher = motorStatsTable.getDoubleTopic("velocity").publish();
        accelerationPublisher = motorStatsTable.getDoubleTopic("acceleration").publish();
        appliedVoltsPublisher = motorStatsTable.getDoubleTopic("appliedVolts").publish();
        supplyCurrentPublisher = motorStatsTable.getDoubleTopic("supplyCurrent").publish();
        statorCurrentPublisher = motorStatsTable.getDoubleTopic("statorCurrent").publish();
        torqueCurrentPublisher = motorStatsTable.getDoubleTopic("torqueCurrent").publish();
        temperaturePublisher = motorStatsTable.getDoubleTopic("temperature").publish();
        dutyCyclePublisher = motorStatsTable.getDoubleTopic("dutyCycle").publish();

        // Target publishers
        targetPositionPublisher = motorStatsTable.getDoubleTopic("targetPosition").publish();
        targetVelocityPublisher = motorStatsTable.getDoubleTopic("targetVelocity").publish();
        targetVoltagePublisher = motorStatsTable.getDoubleTopic("targetVoltage").publish();
        targetDutyCyclePublisher = motorStatsTable.getDoubleTopic("targetDutyCycle").publish();
        targetTorqueCurrentPublisher = motorStatsTable.getDoubleTopic("targetTorqueCurrent").publish();

        // Closed loop publishers
        closedLoopErrorPublisher = motorStatsTable.getDoubleTopic("closedLoop/error").publish();
        closedLoopOutputPublisher = motorStatsTable.getDoubleTopic("closedLoop/output").publish();
        closedLoopReferencePublisher = motorStatsTable.getDoubleTopic("closedLoop/reference").publish();
        closedLoopPOutputPublisher = motorStatsTable.getDoubleTopic("closedLoop/pOutput").publish();
        closedLoopIOutputPublisher = motorStatsTable.getDoubleTopic("closedLoop/iOutput").publish();
        closedLoopDOutputPublisher = motorStatsTable.getDoubleTopic("closedLoop/dOutput").publish();
        closedLoopFFOutputPublisher = motorStatsTable.getDoubleTopic("closedLoop/ffOutput").publish();

        // Fault publishers
        faultHardwarePublisher = motorStatsTable.getBooleanTopic("faults/hardware").publish();
        faultBootDuringEnablePublisher = motorStatsTable.getBooleanTopic("faults/bootDuringEnable").publish();
        faultDeviceTempPublisher = motorStatsTable.getBooleanTopic("faults/deviceTemp").publish();
        faultProcTempPublisher = motorStatsTable.getBooleanTopic("faults/procTemp").publish();
        faultUndervoltagePublisher = motorStatsTable.getBooleanTopic("faults/undervoltage").publish();
        faultSupplyCurrLimitPublisher = motorStatsTable.getBooleanTopic("faults/supplyCurrLimit").publish();
        faultStatorCurrLimitPublisher = motorStatsTable.getBooleanTopic("faults/statorCurrLimit").publish();
    }

    private void initLogs(String name) {
        String prefix = name + "/";

        // State log entries
        positionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "position");
        velocityLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "velocity");
        accelerationLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "acceleration");
        appliedVoltsLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "appliedVolts");
        supplyCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "supplyCurrent");
        statorCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "statorCurrent");
        torqueCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "torqueCurrent");
        temperatureLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "temperature");
        dutyCycleLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "dutyCycle");

        // Target log entries
        targetPositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "targetPosition");
        targetVelocityLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "targetVelocity");
        targetVoltageLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "targetVoltage");
        targetDutyCycleLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "targetDutyCycle");
        targetTorqueCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "targetTorqueCurrent");

        // Closed loop log entries
        closedLoopErrorLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "closedLoop/error");
        closedLoopOutputLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "closedLoop/output");
        closedLoopReferenceLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "closedLoop/reference");
        closedLoopPOutputLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "closedLoop/pOutput");
        closedLoopIOutputLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "closedLoop/iOutput");
        closedLoopDOutputLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "closedLoop/dOutput");
        closedLoopFFOutputLogEntry = new DoubleLogEntry(DataLogManager.getLog(), prefix + "closedLoop/ffOutput");

        // Fault log entries
        faultHardwareLogEntry = new BooleanLogEntry(DataLogManager.getLog(), prefix + "faults/hardware");
        faultBootDuringEnableLogEntry = new BooleanLogEntry(DataLogManager.getLog(), prefix + "faults/bootDuringEnable");
        faultDeviceTempLogEntry = new BooleanLogEntry(DataLogManager.getLog(), prefix + "faults/deviceTemp");
        faultProcTempLogEntry = new BooleanLogEntry(DataLogManager.getLog(), prefix + "faults/procTemp");
        faultUndervoltageLogEntry = new BooleanLogEntry(DataLogManager.getLog(), prefix + "faults/undervoltage");
        faultSupplyCurrLimitLogEntry = new BooleanLogEntry(DataLogManager.getLog(), prefix + "faults/supplyCurrLimit");
        faultStatorCurrLimitLogEntry = new BooleanLogEntry(DataLogManager.getLog(), prefix + "faults/statorCurrLimit");
    }

    /**
     * Allows changing PID through NT
     */
    public void enableDebug() {
        motorStatsTable.getDoubleArrayTopic("PIDSVG").publish().set(pidsvg);
        motorStatsTable.addListener("PIDSVG",
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                configurePIDSVG(
                    event.valueData.value.getDoubleArray()[0],
                    event.valueData.value.getDoubleArray()[1],
                    event.valueData.value.getDoubleArray()[2],
                    event.valueData.value.getDoubleArray()[3],
                    event.valueData.value.getDoubleArray()[4],
                    event.valueData.value.getDoubleArray()[5]
                );
            }
        );
    }

    // ==================== GETTERS ====================

    /**
     * Refreshes all cached status signals. Call this once per loop before
     * reading values for best performance.
     */
    public void refreshSignals() {
        StatusSignal.refreshAll(
            positionSignal,
            velocitySignal,
            accelerationSignal,
            motorVoltageSignal,
            supplyCurrentSignal,
            statorCurrentSignal,
            torqueCurrentSignal,
            temperatureSignal,
            dutyCycleSignal,
            closedLoopErrorSignal,
            closedLoopOutputSignal,
            closedLoopReferenceSignal,
            closedLoopPOutputSignal,
            closedLoopIOutputSignal,
            closedLoopDOutputSignal,
            closedLoopFeedForwardSignal
        );
    }

    /**
     * Gets motor's position in rotations
     * @return position of the motor in rotations
     */
    public double getPosition() {
        return positionSignal.getValue().in(Rotation);
    }

    /**
     * Gets motor's velocity in RPS
     * @return velocity of the motor in RPS
     */
    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    /**
     * Gets motor's acceleration in RPS^2
     * @return acceleration of the motor
     */
    public double getAcceleration() {
        return accelerationSignal.getValueAsDouble();
    }

    /**
     * Gets motor's closed loop error
     * @return closed loop error
     */
    public double getClosedLoopError() {
        return closedLoopErrorSignal.getValueAsDouble();
    }

    /**
     * Gets the applied motor voltage
     * @return applied voltage
     */
    public double getAppliedVoltage() {
        return motorVoltageSignal.getValueAsDouble();
    }

    /**
     * Gets the supply current
     * @return supply current in amps
     */
    public double getSupplyCurrent() {
        return supplyCurrentSignal.getValueAsDouble();
    }

    /**
     * Gets the stator current
     * @return stator current in amps
     */
    public double getStatorCurrent() {
        return statorCurrentSignal.getValueAsDouble();
    }

    /**
     * Gets the torque current
     * @return torque current in amps
     */
    public double getTorqueCurrent() {
        return torqueCurrentSignal.getValueAsDouble();
    }

    /**
     * Gets the device temperature
     * @return temperature in Celsius
     */
    public double getTemperature() {
        return temperatureSignal.getValueAsDouble();
    }

    /**
     * Gets the underlying TalonFX motor controller
     * @return the TalonFX instance
     */
    public TalonFX getMotor() {
        return motor;
    }

    // ==================== LOGGING ====================

    /**
     * Publishes motor stats to NT for real-time viewing
     */
    public void publishStats() {
        // State values
        positionPublisher.set(positionSignal.getValueAsDouble());
        velocityPublisher.set(velocitySignal.getValueAsDouble());
        accelerationPublisher.set(accelerationSignal.getValueAsDouble());
        appliedVoltsPublisher.set(motorVoltageSignal.getValueAsDouble());
        supplyCurrentPublisher.set(supplyCurrentSignal.getValueAsDouble());
        statorCurrentPublisher.set(statorCurrentSignal.getValueAsDouble());
        torqueCurrentPublisher.set(torqueCurrentSignal.getValueAsDouble());
        temperaturePublisher.set(temperatureSignal.getValueAsDouble());
        dutyCyclePublisher.set(dutyCycleSignal.getValueAsDouble());

        // Target values
        targetPositionPublisher.set(targetPosition);
        targetVelocityPublisher.set(targetVelocity);
        targetVoltagePublisher.set(targetVoltage);
        targetDutyCyclePublisher.set(targetDutyCycle);
        targetTorqueCurrentPublisher.set(targetTorqueCurrent);

        // Closed loop values
        closedLoopErrorPublisher.set(closedLoopErrorSignal.getValueAsDouble());
        closedLoopOutputPublisher.set(closedLoopOutputSignal.getValueAsDouble());
        closedLoopReferencePublisher.set(closedLoopReferenceSignal.getValueAsDouble());
        closedLoopPOutputPublisher.set(closedLoopPOutputSignal.getValueAsDouble());
        closedLoopIOutputPublisher.set(closedLoopIOutputSignal.getValueAsDouble());
        closedLoopDOutputPublisher.set(closedLoopDOutputSignal.getValueAsDouble());
        closedLoopFFOutputPublisher.set(closedLoopFeedForwardSignal.getValueAsDouble());

        // Fault values
        faultHardwarePublisher.set(motor.getFault_Hardware().getValue());
        faultBootDuringEnablePublisher.set(motor.getFault_BootDuringEnable().getValue());
        faultDeviceTempPublisher.set(motor.getFault_DeviceTemp().getValue());
        faultProcTempPublisher.set(motor.getFault_ProcTemp().getValue());
        faultUndervoltagePublisher.set(motor.getFault_Undervoltage().getValue());
        faultSupplyCurrLimitPublisher.set(motor.getFault_SupplyCurrLimit().getValue());
        faultStatorCurrLimitPublisher.set(motor.getFault_StatorCurrLimit().getValue());
    }

    /**
     * Logs motor stats into log file
     */
    public void logStats() {
        long timestamp = GRTUtil.getFPGATime();

        // State values
        positionLogEntry.append(positionSignal.getValueAsDouble(), timestamp);
        velocityLogEntry.append(velocitySignal.getValueAsDouble(), timestamp);
        accelerationLogEntry.append(accelerationSignal.getValueAsDouble(), timestamp);
        appliedVoltsLogEntry.append(motorVoltageSignal.getValueAsDouble(), timestamp);
        supplyCurrentLogEntry.append(supplyCurrentSignal.getValueAsDouble(), timestamp);
        statorCurrentLogEntry.append(statorCurrentSignal.getValueAsDouble(), timestamp);
        torqueCurrentLogEntry.append(torqueCurrentSignal.getValueAsDouble(), timestamp);
        temperatureLogEntry.append(temperatureSignal.getValueAsDouble(), timestamp);
        dutyCycleLogEntry.append(dutyCycleSignal.getValueAsDouble(), timestamp);

        // Target values
        targetPositionLogEntry.append(targetPosition, timestamp);
        targetVelocityLogEntry.append(targetVelocity, timestamp);
        targetVoltageLogEntry.append(targetVoltage, timestamp);
        targetDutyCycleLogEntry.append(targetDutyCycle, timestamp);
        targetTorqueCurrentLogEntry.append(targetTorqueCurrent, timestamp);

        // Closed loop values
        closedLoopErrorLogEntry.append(closedLoopErrorSignal.getValueAsDouble(), timestamp);
        closedLoopOutputLogEntry.append(closedLoopOutputSignal.getValueAsDouble(), timestamp);
        closedLoopReferenceLogEntry.append(closedLoopReferenceSignal.getValueAsDouble(), timestamp);
        closedLoopPOutputLogEntry.append(closedLoopPOutputSignal.getValueAsDouble(), timestamp);
        closedLoopIOutputLogEntry.append(closedLoopIOutputSignal.getValueAsDouble(), timestamp);
        closedLoopDOutputLogEntry.append(closedLoopDOutputSignal.getValueAsDouble(), timestamp);
        closedLoopFFOutputLogEntry.append(closedLoopFeedForwardSignal.getValueAsDouble(), timestamp);

        // Fault values
        faultHardwareLogEntry.append(motor.getFault_Hardware().getValue(), timestamp);
        faultBootDuringEnableLogEntry.append(motor.getFault_BootDuringEnable().getValue(), timestamp);
        faultDeviceTempLogEntry.append(motor.getFault_DeviceTemp().getValue(), timestamp);
        faultProcTempLogEntry.append(motor.getFault_ProcTemp().getValue(), timestamp);
        faultUndervoltageLogEntry.append(motor.getFault_Undervoltage().getValue(), timestamp);
        faultSupplyCurrLimitLogEntry.append(motor.getFault_SupplyCurrLimit().getValue(), timestamp);
        faultStatorCurrLimitLogEntry.append(motor.getFault_StatorCurrLimit().getValue(), timestamp);
    }
}
