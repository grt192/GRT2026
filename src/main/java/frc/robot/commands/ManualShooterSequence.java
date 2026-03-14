package frc.robot.commands;

import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.subsystems.hopper.HopperSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual shooter sequence - no auto-aim.
 * Uses fixed hood position and flywheel RPS from SmashAndShootConstants.
 * Waits for flywheel and hood to reach targets before feeding balls.
 */
public class ManualShooterSequence extends Command {

    private final flywheel fly;
    private final towerRollers tower;
    private final HopperSubsystem hopper;

    public ManualShooterSequence(
        flywheel fly,
        towerRollers tower,
        HopperSubsystem hopper) {
        this.fly = fly;
        this.tower = tower;
        this.hopper = hopper;

        addRequirements(fly, tower, hopper);
    }

    @Override
    public void initialize() {
        // Start ramping flywheel and moving hood to position
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);
    }

    @Override
    public void execute() {
        // Keep commanding flywheel and hood targets
        fly.shoot(SmashAndShootConstants.FLYWHEEL_RPS);

        // Only feed balls when flywheel is at speed AND hood is at position
        if (fly.wantedVel()) {
            tower.setManualControl(SmashAndShootConstants.TOWER_DUTY_CYCLE);
            hopper.setManualControl(SmashAndShootConstants.INDEXER_DUTY_CYCLE);
        } else {
            tower.setManualControl(0);
            hopper.setManualControl(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        fly.dontShoot();
        tower.setManualControl(0);
        hopper.setManualControl(0);
    }
}
