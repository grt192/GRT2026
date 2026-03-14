package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.DebugConstants.LOG_TO_NT;
import java.util.EnumSet;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class flywheel extends SubsystemBase {
    private NetworkTable NTtable;

    private final LoggedTalon upperMotor;
    private final LoggedTalon secondMotor;
    private MotionMagicVelocityVoltage spinner = new MotionMagicVelocityVoltage(0);
    private DutyCycleOut dutyCycl = new DutyCycleOut(0);
    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private Slot0Configs pidSlots = new Slot0Configs();

    private final VoltageOut sysIdVoltage = new VoltageOut(0);
    private final SysIdRoutine sysIdRoutine;

    private double wantedVe = 0;

    private static final String LOG_PREFIX = "FlyWheel/";

    private void yoTuneThis(String valueName, Consumer<Double> configSetter, double defaultVal) {
        NTtable.getEntry(valueName).setDouble(defaultVal);
        NTtable.addListener(valueName, EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
            configSetter.accept(event.valueData.value.getDouble());

            cfg.withSlot0(pidSlots);
            upperMotor.getConfigurator().apply(cfg);
            secondMotor.getConfigurator().apply(cfg);

            System.out.println("Updated: " + valueName + " to this: " + event.valueData.value.getDouble() + "!");
        });
    }

    private void configThruNT() {
        NTtable = NetworkTableInstance.getDefault().getTable("tuneFlywheel");
        yoTuneThis("Pids/P", val -> pidSlots.withKP(val), ShooterConstants.Flywheel.KP);
        yoTuneThis("Pids/I", val -> pidSlots.withKI(val), ShooterConstants.Flywheel.KI);
        yoTuneThis("Pids/D", val -> pidSlots.withKD(val), ShooterConstants.Flywheel.KD);
        yoTuneThis("Pids/S", val -> pidSlots.withKS(val), ShooterConstants.Flywheel.KS);
        yoTuneThis("Pids/V", val -> pidSlots.withKV(val), ShooterConstants.Flywheel.KV);
        yoTuneThis("setDutyCyclePercent", val -> upperMotor.setControl(new DutyCycleOut(val)), 0);
        yoTuneThis("setMMVTCF", val -> upperMotor.setControl(new VelocityVoltage(val)), 0);

        yoTuneThis("MMAccel", val -> cfg.MotionMagic.MotionMagicAcceleration = val, ShooterConstants.Flywheel.MM_ACCEL);
        yoTuneThis("MMJerk", val -> cfg.MotionMagic.MotionMagicJerk = val, ShooterConstants.Flywheel.MM_JERK);
        yoTuneThis("MMMaxVelo", val -> cfg.MotionMagic.MotionMagicCruiseVelocity = val, ShooterConstants.Flywheel.MM_CRUISE_VELOCITY);

        yoTuneThis("GearReduction", val -> cfg.Feedback.SensorToMechanismRatio = val, ShooterConstants.Flywheel.GEAR_RATIO);
        yoTuneThis("printThisYo", val -> System.out.println("printed this yo: " + val), 0);
    }

    public flywheel(CANBus cn) {
        upperMotor = new LoggedTalon(ShooterConstants.Flywheel.UPPER_MOTOR_ID, cn, "FlywheelUpperMotor");
        secondMotor = new LoggedTalon(ShooterConstants.Flywheel.SECOND_MOTOR_ID, cn, "FlywheelLowerMotor");

        config();
        if (LOG_TO_NT) {
            configThruNT();
        }

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds), // ramp rate: 1 V/s
                Volts.of(7), // step voltage
                Seconds.of(10) // timeout
            ),
            new SysIdRoutine.Mechanism(
                voltage -> upperMotor.setControl(sysIdVoltage.withOutput(voltage.in(Volts))),
                log -> {
                    log.motor("flywheel")
                        .voltage(Volts.of(upperMotor.getMotorVoltage().getValueAsDouble()))
                        .angularPosition(Rotations.of(upperMotor.getPosition().getValueAsDouble()))
                        .angularVelocity(RotationsPerSecond.of(upperMotor.getVelocity().getValueAsDouble()));
                },
                this));
    }

    public void config() {
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.Flywheel.MM_CRUISE_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.Flywheel.MM_ACCEL;
        cfg.MotionMagic.MotionMagicJerk = ShooterConstants.Flywheel.MM_JERK;

        pidSlots.withKP(ShooterConstants.Flywheel.KP);
        pidSlots.withKI(ShooterConstants.Flywheel.KI);
        pidSlots.withKD(ShooterConstants.Flywheel.KD);
        pidSlots.withKS(ShooterConstants.Flywheel.KS);
        pidSlots.withKV(ShooterConstants.Flywheel.KV);
        cfg.withSlot0(pidSlots);

        cfg.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEAR_RATIO;

        upperMotor.getConfigurator().apply(cfg);
        secondMotor.getConfigurator().apply(cfg);

        secondMotor.setControl(new Follower(ShooterConstants.Flywheel.UPPER_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void shoot(double rps) {
        wantedVe = rps;
        upperMotor.setControl(spinner.withVelocity(rps));
    }

    public double getRPS() {
        return upperMotor.getVelocity().getValueAsDouble();
    }

    public boolean wantedVel() {
        return wantedVe > 0 && Math.abs(wantedVe - upperMotor.getVelocity().getValueAsDouble()) < ShooterConstants.Flywheel.VELOCITY_TOLERANCE_RPS;
    }

    public void dontShoot() {
        wantedVe = 0;
        upperMotor.setControl(spinner.withVelocity(0));
    }

    double commandedDutyCycle = 0;

    public void flySpeed(double speed) {
        if (speed > 0.75) {
            commandedDutyCycle = 0.65;
            upperMotor.setControl(dutyCycl.withOutput(commandedDutyCycle));
        } else {
            commandedDutyCycle = 0.0;
            upperMotor.setControl(dutyCycl.withOutput(0.0));
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        upperMotor.updateDashboard();
        secondMotor.updateDashboard();
    }
}
