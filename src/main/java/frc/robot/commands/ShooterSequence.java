package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.shooter.towerRollers;

import frc.robot.commands.shooter.flywheel.rampFlywheel;
import frc.robot.commands.shooter.hood.hoodCommand;
import frc.robot.commands.tower.towerRoll;
import frc.robot.commands.hopper.indexerRun;

import frc.robot.commands.allign.allignCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterSequence extends SequentialCommandGroup {

    public ShooterSequence(SwerveSubsystem swerve, flywheel fly, hood hood, HopperSubsystem hopper, towerRollers tower, VisionSubsystem vis) {

        addCommands(

            // Aim first
            new allignCommand(swerve)
                .until(() -> swerve.isAligned()),

            // Then shoot
            new ParallelDeadlineGroup(
                // Deadline: stop when hopper is empty
                new WaitUntilCommand(() -> !vis.hasBall()),

                // Always running
                new rampFlywheel(fly, swerve),
                new hoodCommand(hood, swerve),

                // Only feed when ready
                new ConditionalCommand(
                    new indexerRun(hopper),
                    new towerRoll(tower),
                    () -> fly.wantedVel() && hood.wantedAngl()
                )
            )
        );
    }
}