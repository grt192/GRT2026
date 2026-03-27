package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class PivotIntakeSubsystem extends SubsystemBase {
    private final PivotIntakeIO io;
    private final PivotIntakeIOInputsAutoLogged inputs = new PivotIntakeIOInputsAutoLogged();

    public PivotIntakeSubsystem(PivotIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Pivot", inputs);
    }

    public double getAngleDegrees() {
        return inputs.encoderPositionRotations * 360.0;
    }

    public double getAbsolutePosition() {
        return inputs.encoderAbsolutePosition;
    }

    public double getMotorPosition() {
        return inputs.motorPositionRotations;
    }

    public void setPosition(double rotations) {
        io.setPosition(rotations);
    }

    public void zeroEncoder() {
        io.zeroEncoder();
    }

    public void setEncoderToMax() {
        io.setEncoderToMax();
    }

    public void setManualSpeed(double speed) {
        io.setManualSpeed(speed);
    }

    public void stop() {
        io.stop();
    }
}
