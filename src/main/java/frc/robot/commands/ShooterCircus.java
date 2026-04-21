package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.towerRollers;

import frc.robot.commands.shooter.rampFlywheel;
import frc.robot.commands.shooter.hoodCommand;
import frc.robot.commands.hopper.indexerRun;
import frc.robot.commands.shooter.towerRollers.towerRoll;
import frc.robot.commands.allign.AimWhileDrivingCommand;
import frc.robot.commands.intake.pivot.PivotToggleCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.function.DoubleSupplier;

public class ShooterCircus extends SequentialCommandGroup {

    public ShooterCircus(
        SwerveSubsystem swerve,
        flywheel fly,
        hood hood,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake,
        FieldManagementSubsystem fms,
        towerRollers b,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed) {

        boolean redTeam = fms.isRedAlliance();

        addCommands(

            new ParallelCommandGroup(
                new PivotToggleCommand(pivotIntake),
                new rampFlywheel(fly),
                new hoodCommand(hood),
                new towerRoll(b),
                new indexerRun(hopper)).withTimeout(3.0),

            Commands.runEnd(
                () -> swerve.setRobotRelativeDrivePowers(0.25, 0.0, 0.0),
                () -> swerve.setRobotRelativeDrivePowers(0.0, 0.0, 0.0),
                swerve).withTimeout(0.5));
    }
}
