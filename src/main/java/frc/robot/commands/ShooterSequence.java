public class ShooterSequence extends SequentialCommandGroup {
    public ShooterSequence(Drive drive, Shooter shooter,
                           Hood hood, Indexer indexer, Vision vision) {

        addCommands(
            new AimCommand(drive, vision)
                .until(() -> vision.isAligned()),

            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !hopper.hasBall()), // stop when empty
                new RampFlywheel(shooter),
                new SetHoodAngle(hood),
                new ConditionalCommand(
                    new RunIndexer(indexer),
                    Commands.none(),
                    () -> shooter.atSpeed() && hood.atSetpoint()
                )
            )
        );
    }
}
