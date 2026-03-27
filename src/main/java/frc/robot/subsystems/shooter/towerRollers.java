package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;
import org.littletonrobotics.junction.Logger;

public class towerRollers extends SubsystemBase {
    private final TowerRollersIO io;
    private final TowerRollersIOInputsAutoLogged inputs = new TowerRollersIOInputsAutoLogged();

    public towerRollers(TowerRollersIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TowerRollers", inputs);
    }

    public void setTower(TOWER_INTAKE state) {
        switch (state) {
            case BALLUP:
                io.setVelocity(TowerConstants.TARGET_RPS);
                break;
            case BALLDOWN:
                io.setVelocity(-TowerConstants.TARGET_RPS);
                break;
            case STOP:
                io.stop();
                break;
        }
    }

    public boolean correctRoll() {
        return Math.abs(TowerConstants.TARGET_RPS - inputs.velocityRPS) < 2;
    }

    public void setManualControl(double percentOutput) {
        io.setDutyCycle(percentOutput);
    }
}
