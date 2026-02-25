package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterSequence extends SequentialCommandGroup {
    public ShooterSequence(Drive drive, flywheel fly,
                           hood hood, Indexer indexer, Vision vision, Tower tower) {

        addCommands(
            new AimCommand(drive, vision)
                .until(() -> vision.isAligned()),

            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !hopper.hasBall()), // stop when empty
                new RampFlywheel(fly),
                new SetHoodAngle(hood),
                new ConditionalCommand(
                    new RunIndexer(indexer),
                    new RunTower(tower);
                    Commands.none(),
                    () -> shooter.atSpeed() && hood.atSetpoint()
                )
            )
        );
    }
}
