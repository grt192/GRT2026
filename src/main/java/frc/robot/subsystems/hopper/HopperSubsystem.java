package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperConstants.HOPPER_INTAKE;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public HopperSubsystem(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public void setHopper(HOPPER_INTAKE state) {
        switch (state) {
            case BALLIN:
                io.setVelocity(HopperConstants.TARGET_RPS);
                break;
            case BALLOUT:
                io.setVelocity(-HopperConstants.TARGET_RPS);
                break;
            case STOP:
                io.stop();
                break;
        }
    }

    public void setManualControl(double percentOutput) {
        io.setDutyCycle(percentOutput);
    }
}
