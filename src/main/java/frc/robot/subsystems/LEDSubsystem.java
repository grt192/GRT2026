package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
                return LEFT_HOPPER_START_END;
            case RIGHT_HOPPER:
                return RIGHT_HOPPER_START_END;
            case ALL:
                return new int[] {
                        Math.min(LEFT_HOPPER_START_END[0], RIGHT_HOPPER_START_END[0]),
                        Math.max(LEFT_HOPPER_START_END[1], RIGHT_HOPPER_START_END[1])
                };
            default:
                return new int[] {0, 0};
        }
    }
    }
}
