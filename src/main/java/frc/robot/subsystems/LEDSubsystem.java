package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle;

    public LEDSubsystem(CANBus can) {
        candle = new CANdle(0, can);
    }
}
