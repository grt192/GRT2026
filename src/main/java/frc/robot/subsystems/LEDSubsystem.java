package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    public enum LedStrip {
        LEFT_HOPPER,
        RIGHT_HOPPER,
        ALL
    }

    public enum CANAlert {
        SWERVE,
        MECH,
        BOTH
    }

    private CANdle candle;
    private CANdleConfiguration candleConfig;

    public LEDSubsystem(CANBus can) {
        candle = new CANdle(0, can);

        configureCANdle();
    }

    private void configureCANdle() {
        candleConfig = new CANdleConfiguration()
            .withLED(new LEDConfigs()
                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs)
                .withStripType(StripTypeValue.RGB))
            .withCANdleFeatures(new CANdleFeaturesConfigs()
                .withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled));

        candle.getConfigurator().apply(candleConfig);
    }

    private int[] getLEDIndexes(LedStrip strip) {
        switch (strip) {
            case LEFT_HOPPER:
                return LEDConstants.LEFT_HOPPER_START_END;
            case RIGHT_HOPPER:
                return LEDConstants.RIGHT_HOPPER_START_END;
            case ALL:
                return new int[] {
                        Math.min(LEDConstants.LEFT_HOPPER_START_END[0], LEDConstants.RIGHT_HOPPER_START_END[0]),
                        Math.max(LEDConstants.LEFT_HOPPER_START_END[1], LEDConstants.RIGHT_HOPPER_START_END[1])
                };
            default:
                return new int[] {0, 0};
        }
    }

    private void clearStrip(LedStrip strip) {
        if (strip == LedStrip.ALL) {
            candle.setControl(new EmptyAnimation(LedStrip.LEFT_HOPPER.ordinal()));
            candle.setControl(new EmptyAnimation(LedStrip.RIGHT_HOPPER.ordinal()));
        }
        candle.setControl(new EmptyAnimation(strip.ordinal()));
    }

    public StrobeAnimation getStrobeControl(LedStrip strip, RGBWColor color) {
        int[] indexes = getLEDIndexes(strip);
        return new StrobeAnimation(indexes[0], indexes[1])
            .withSlot(strip.ordinal())
            .withColor(color);
    }

    public Command strobeCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getStrobeControl(strip, color));
        });
    }

    public SingleFadeAnimation getFadeControl(LedStrip strip, RGBWColor color) {
        int[] indexes = getLEDIndexes(strip);
        return new SingleFadeAnimation(indexes[0], indexes[1])
            .withSlot(strip.ordinal())
            .withColor(color);
    }

    public Command fadeCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getFadeControl(strip, color));
        });
    }

    public SolidColor getSolidColorControl(LedStrip strip, RGBWColor color) {
        int[] indexes = getLEDIndexes(strip);
        return new SolidColor(indexes[0], indexes[1])
            .withColor(color);
    }

    public Command solidColorCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getSolidColorControl(strip, color));
        });
    }

    public LarsonAnimation getBounceControl(LedStrip strip, RGBWColor color, int windowSize) {
        int[] indexes = getLEDIndexes(strip);
        return new LarsonAnimation(indexes[0], indexes[1])
            .withSlot(strip.ordinal())
            .withColor(color)
            .withBounceMode(LarsonBounceValue.Front)
            .withSize(windowSize);
    }

    public Command bounceCommand(LedStrip strip, RGBWColor color, int windowSize) {
        return this.runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getBounceControl(strip, color, windowSize));
        });
    }

    public ColorFlowAnimation getFlowControl(LedStrip strip, RGBWColor color) {
        int[] indexes = getLEDIndexes(strip);
        return new ColorFlowAnimation(indexes[0], indexes[1])
            .withSlot(strip.ordinal())
            .withColor(color);
    }

    public Command flowingCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getFlowControl(strip, color));
        });
    }

    public ColorFlowAnimation getLoadingControl(LedStrip strip, RGBWColor color, Time fillTime) {
        int[] indexes = getLEDIndexes(strip);
        Frequency animationFrequency = fillTime.div(indexes[1] - indexes[0] + 1).asFrequency();
        return new ColorFlowAnimation(indexes[0], indexes[1])
            .withSlot(strip.ordinal())
            .withColor(color)
            .withFrameRate(animationFrequency);
    }

    public Command loadingCommand(LedStrip strip, RGBWColor color, Time fillTime) {
        return this.runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getLoadingControl(strip, color, fillTime));
        }).withTimeout(fillTime).andThen(solidColorCommand(strip, color));
    }

    public Command getDisabledCommand(boolean mechCanHealthy, boolean swerveCanHealthy) {
        boolean bothFucked = !mechCanHealthy && !swerveCanHealthy;
        if (bothFucked) {
            return bounceCommand(LedStrip.ALL, LEDConstants.WHITE, 5);
        } else if (!mechCanHealthy) {
            return bounceCommand(LedStrip.ALL, LEDConstants.ORANGE, 5);
        } else if (!swerveCanHealthy) {
            return bounceCommand(LedStrip.ALL, LEDConstants.PURPLE, 5);
        } else { // ALL GOOD
            Alliance allianceSide = DriverStation.getAlliance().orElse(Alliance.Red); // Defaults to red (GRT Color!!!)
            RGBWColor allianceColor = switch (allianceSide) {
                case Red -> LEDConstants.RED_ALLIANCE;
                case Blue -> LEDConstants.BLUE_ALLIANCE;
            };
            return fadeCommand(LedStrip.ALL, allianceColor);
        }
    }

    public Command idleAnimation(BooleanSupplier mechCanHealth, BooleanSupplier swerveCanHealth) {
        return new ConditionalCommand(
            Commands.none(), // if enabled; TODO: implement in match behavior. Vans idea to display hub status
            getDisabledCommand(false, false), // If not enabled
            DriverStation::isEnabled).ignoringDisable(true).repeatedly();
    }


}
