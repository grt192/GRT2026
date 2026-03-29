package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    public enum LedStrip {
        LEFT_HOPPER,
        RIGHT_HOPPER,
        ALL
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
        return runOnce(() -> {
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
        return runOnce(() -> {
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
        return runOnce(() -> {
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
        return runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getBounceControl(strip, color, windowSize));
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
        return runOnce(() -> {
            clearStrip(strip);
            candle.setControl(getLoadingControl(strip, color, fillTime));
        });
    }
}
