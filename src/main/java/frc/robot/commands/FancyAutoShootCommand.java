package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.allign.AimToHubCommand;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FancyAutoShootCommand extends SequentialCommandGroup {

    private AimToHubCommand aimToHub;

    public FancyAutoShootCommand(SwerveSubsystem swerveSubsystem,
        FieldManagementSubsystem fms,
        flywheel shooterFlywheel,
        hood shooterHood,
        towerRollers tower,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivot) {

        aimToHub = new AimToHubCommand(swerveSubsystem, fms);
        addCommands(
            aimToHub.createAimCommand(() -> false),
            new FancyShooterSequence(shooterFlywheel, shooterHood, tower, hopper, pivot, fms));
    }
}
