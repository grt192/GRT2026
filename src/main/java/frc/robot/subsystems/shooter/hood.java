package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private double wantedAngle = 0.1;

    public hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Hood", inputs);
    }

    public void setHoodAngle(double rotationAngle) {
        io.setPosition(rotationAngle);
        wantedAngle = rotationAngle;
    }

    public boolean wantedAngl() {
        return Math.abs(wantedAngle - inputs.positionRotations) < ShooterConstants.Hood.ANGLE_TOLERANCE;
    }

    public void hoodSpeed(double speed) {
        io.setDutyCycle(speed);
    }

    public double getPos() {
        return inputs.positionRotations;
    }
}
