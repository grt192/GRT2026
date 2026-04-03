package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class RioLEDSubsystem extends SubsystemBase {
    public enum LedStrip {
        LEFT_HOPPER,
        RIGHT_HOPPER,
        ALL
    }

    private static final int LED_LENGTH = LEDConstants.RIGHT_HOPPER_START_END[1] + 1;

    @FunctionalInterface
    private interface AnimationUpdater {
        void update(double time);
    }

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private AnimationUpdater currentAnimation = null;

    public RioLEDSubsystem(int pwmPort) {
        led = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(LED_LENGTH);
        led.start();
    }

    @Override
    public void periodic() {
        if (currentAnimation != null) {
            currentAnimation.update(Timer.getFPGATimestamp());
            led.setData(buffer);
        }
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

    private void setRGB(int index, int r, int g, int b) {
        buffer.setRGB(index, r, g, b);
    }

    private void setSolidRange(int start, int end, RGBWColor color) {
        for (int i = start; i <= end; i++) {
            setRGB(i, color.Red, color.Green, color.Blue);
        }
    }

    private void clearRange(int start, int end) {
        for (int i = start; i <= end; i++) {
            setRGB(i, 0, 0, 0);
        }
    }

    private void clearStrip(LedStrip strip) {
        int[] indexes = getLEDIndexes(strip);
        clearRange(indexes[0], indexes[1]);
    }

    public Command solidColorCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            int[] indexes = getLEDIndexes(strip);
            currentAnimation = (time) -> setSolidRange(indexes[0], indexes[1], color);
        });
    }

    public Command strobeCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            int[] indexes = getLEDIndexes(strip);
            currentAnimation = (time) -> {
                boolean on = ((int) (time * 10)) % 2 == 0;
                if (on) {
                    setSolidRange(indexes[0], indexes[1], color);
                } else {
                    clearRange(indexes[0], indexes[1]);
                }
            };
        });
    }

    public Command fadeCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            int[] indexes = getLEDIndexes(strip);
            currentAnimation = (time) -> {
                double brightness = (Math.sin(time * 2 * Math.PI * 0.5) + 1.0) / 2.0;
                for (int i = indexes[0]; i <= indexes[1]; i++) {
                    setRGB(i,
                        (int) (color.Red * brightness),
                        (int) (color.Green * brightness),
                        (int) (color.Blue * brightness));
                }
            };
        });
    }

    public Command bounceCommand(LedStrip strip, RGBWColor color, int windowSize) {
        return this.runOnce(() -> {
            int[] indexes = getLEDIndexes(strip);
            int stripLen = indexes[1] - indexes[0] + 1;
            currentAnimation = (time) -> {
                clearRange(indexes[0], indexes[1]);
                int range = stripLen - windowSize;
                if (range <= 0) {
                    setSolidRange(indexes[0], indexes[1], color);
                    return;
                }
                double pos = (Math.sin(time * 2 * Math.PI * 0.75) + 1.0) / 2.0 * range;
                int start = indexes[0] + (int) pos;
                int end = Math.min(start + windowSize - 1, indexes[1]);
                setSolidRange(start, end, color);
            };
        });
    }

    public Command flowingCommand(LedStrip strip, RGBWColor color) {
        return this.runOnce(() -> {
            int[] indexes = getLEDIndexes(strip);
            int stripLen = indexes[1] - indexes[0] + 1;
            currentAnimation = (time) -> {
                int offset = ((int) (time * 10)) % stripLen;
                for (int i = indexes[0]; i <= indexes[1]; i++) {
                    double brightness = (double) ((i - indexes[0] + offset) % stripLen) / stripLen;
                    setRGB(i,
                        (int) (color.Red * brightness),
                        (int) (color.Green * brightness),
                        (int) (color.Blue * brightness));
                }
            };
        });
    }

    public Command loadingCommand(LedStrip strip, RGBWColor color, double fillTimeSeconds) {
        return this.run(() -> {
            int[] indexes = getLEDIndexes(strip);
            int stripLen = indexes[1] - indexes[0] + 1;
            currentAnimation = (time) -> {
                for (int i = indexes[0]; i <= indexes[1]; i++) {
                    double brightness = (double) (i - indexes[0]) / stripLen;
                    setRGB(i,
                        (int) (color.Red * brightness),
                        (int) (color.Green * brightness),
                        (int) (color.Blue * brightness));
                }
            };
        }).withTimeout(fillTimeSeconds).andThen(solidColorCommand(strip, color));
    }

    // Shift boundaries (matchTime counting down):
    // >130: Transition (active), >105: Shift 1, >80: Shift 2, >55: Shift 3, >30: Shift 4, <=30: Endgame (active)
    private static final double[] SHIFT_BOUNDARIES = {130, 105, 80, 55, 30};

    private boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            return true; // Transition
        } else if (matchTime > 105) {
            return shift1Active; // Shift 1
        } else if (matchTime > 80) {
            return !shift1Active; // Shift 2
        } else if (matchTime > 55) {
            return shift1Active; // Shift 3
        } else if (matchTime > 30) {
            return !shift1Active; // Shift 4
        } else {
            return true; // Endgame
        }
    }

    private double getTimeUntilNextShift() {
        double matchTime = DriverStation.getMatchTime();
        for (double boundary : SHIFT_BOUNDARIES) {
            if (matchTime > boundary) {
                return matchTime - boundary;
            }
        }
        return 0;
    }

    public Command hubStatusCommand(LedStrip strip) {
        return this.run(() -> {
            int[] indexes = getLEDIndexes(strip);
            currentAnimation = (time) -> {
                if (isHubActive()) {
                    setSolidRange(indexes[0], indexes[1], LEDConstants.GREEN);
                } else {
                    double timeUntilActive = getTimeUntilNextShift();
                    if (timeUntilActive <= 5.0) {
                        // Blink red when hub activates within 5 seconds
                        boolean on = ((int) (time * 6)) % 2 == 0; // 3 Hz blink
                        if (on) {
                            setSolidRange(indexes[0], indexes[1], LEDConstants.RED_ALLIANCE);
                        } else {
                            clearRange(indexes[0], indexes[1]);
                        }
                    } else {
                        setSolidRange(indexes[0], indexes[1], LEDConstants.RED_ALLIANCE);
                    }
                }
            };
        });
    }

    public Command disabledIdleAnimation(BooleanSupplier mechCanHealthy, BooleanSupplier swerveCanHealthy) {
        return this.runOnce(() -> {
            boolean bothDown = !mechCanHealthy.getAsBoolean() && !swerveCanHealthy.getAsBoolean();
            if (bothDown) {
                bounceCommand(LedStrip.ALL, LEDConstants.WHITE, 5).schedule();
            } else if (!mechCanHealthy.getAsBoolean()) {
                bounceCommand(LedStrip.ALL, LEDConstants.ORANGE, 5).schedule();
            } else if (!swerveCanHealthy.getAsBoolean()) {
                bounceCommand(LedStrip.ALL, LEDConstants.PURPLE, 5).schedule();
            } else {
                Alliance allianceSide = DriverStation.getAlliance().orElse(Alliance.Red);
                RGBWColor allianceColor = switch (allianceSide) {
                    case Red -> LEDConstants.RED_ALLIANCE;
                    case Blue -> LEDConstants.BLUE_ALLIANCE;
                };
                fadeCommand(LedStrip.ALL, allianceColor).schedule();
            }
        });
    }

    public Command idleAnimation(BooleanSupplier mechCanHealth, BooleanSupplier swerveCanHealth) {
        return new ConditionalCommand(
            hubStatusCommand(LedStrip.ALL),
            disabledIdleAnimation(mechCanHealth, swerveCanHealth),
            DriverStation::isEnabled).ignoringDisable(true).repeatedly();
    }
}
