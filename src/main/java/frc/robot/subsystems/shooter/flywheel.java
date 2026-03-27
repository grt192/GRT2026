package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SmashAndShootConstants;
import org.littletonrobotics.junction.Logger;

public class flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private double targetRPS = SmashAndShootConstants.FLYWHEEL_RPS;

    public flywheel(FlywheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Flywheel", inputs);
    }

    public void shoot(double rps) {
        io.setVelocity(targetRPS);
    }

    public double getRPS() {
        return inputs.velocityRPS;
    }

    public boolean wantedVel() {
        return targetRPS > 0 && Math.abs(targetRPS - inputs.velocityRPS) < ShooterConstants.Flywheel.VELOCITY_TOLERANCE_RPS;
    }

    public void dontShoot() {
        io.stop();
    }

    public void flySpeed(double speed) {
        if (speed > 0.1) {
            io.setVelocity(targetRPS);
        } else {
            io.stop();
        }
    }
}
