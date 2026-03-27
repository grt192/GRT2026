package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.SwerveConstants;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(CANBus canBus) {
        pigeon = new Pigeon2(SwerveConstants.PigeonID, canBus);
        pigeon.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawDegrees = pigeon.getYaw().getValueAsDouble();
        inputs.pitchDegrees = pigeon.getPitch().getValueAsDouble();
        inputs.rollDegrees = pigeon.getRoll().getValueAsDouble();
        inputs.yawVelocityDegPerSec = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        inputs.connected = pigeon.getYaw().getStatus().isOK();
        inputs.stickyFaultUndervoltage = pigeon.getStickyFault_Undervoltage().getValue();
    }

    @Override
    public void reset() {
        pigeon.reset();
    }
}
