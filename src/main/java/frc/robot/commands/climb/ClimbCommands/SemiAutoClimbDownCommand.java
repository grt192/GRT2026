package frc.robot.commands.climb.ClimbCommands;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class SemiAutoClimbDownCommand extends SequentialCommandGroup {

    private Command deployArmUntilStop;
    private Command runWinchUpThenRetractArm;

    public SemiAutoClimbDownCommand(ClimbSubsystem climbSubsystem, BooleanSupplier stepSupplier) {
        AtomicBoolean retractArm = new AtomicBoolean(false);

        deployArmUntilStop = Commands.run(() -> climbSubsystem.semiAutoDeployArm())
            .until(() -> retractArm.get())
            .finallyDo((boolean interrupted) -> climbSubsystem.stopArm());

        runWinchUpThenRetractArm = Commands.run(() -> climbSubsystem.manualHomeWinch(), climbSubsystem)
            .until(() -> stepSupplier.getAsBoolean() || climbSubsystem.isWinchAtDistance(ClimbConstants.WINCH_HOME_DISTANCE))
            .finallyDo(() -> climbSubsystem.stopWinch())
            .andThen(waitForNextStep(stepSupplier))
            .andThen(Commands.runOnce(() -> retractArm.set(true)));

        addCommands(
            waitForButtonRelease(stepSupplier),
            deployArmUntilStop.alongWith(runWinchUpThenRetractArm));

        addRequirements(climbSubsystem);
    }

    private static Command waitForButtonRelease(BooleanSupplier supplier) {
        return Commands.waitUntil(() -> !supplier.getAsBoolean());
    }

    private static Command waitForNextStep(BooleanSupplier supplier) {
        return waitForButtonRelease(supplier)
            .andThen(Commands.waitUntil(supplier))
            .andThen(waitForButtonRelease(supplier));
    }
}
