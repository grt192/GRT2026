package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class RollerIntakeSubsystem extends SubsystemBase {
    private final RollerIntakeIO io;
    private final RollerIntakeIOInputsAutoLogged inputs = new RollerIntakeIOInputsAutoLogged();

    public RollerIntakeSubsystem(RollerIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Roller", inputs);
    }

    public void setVelocity(double velocity) {
        io.setVelocity(velocity);
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public void stop() {
        io.stop();
    }

    public void runIn() {
        setVelocity(-Math.abs(IntakeConstants.ROLLER_IN_SPEED));
    }

    public void runOut(double speed) {
        setVelocity(Math.abs(IntakeConstants.ROLLER_OUT_SPEED));
    }

    public void runInDutyCycle() {
        setDutyCycle(-1.0);
    }

    public void runOutDutyCycle() {
        setDutyCycle(1.0);
    }

    public boolean isRunning() {
        return Math.abs(inputs.dutyCycle) > 0.01;
    }
}
