package frc.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;

public class IntakeAndShootAuton extends SequentialCommandGroup {

    public IntakeAndShootAuton(
        PivotIntakeSubsystem pivotIntake,
        RollerIntakeSubsystem rollerIntake,
        flywheel flywheel,
        hood hood,
        towerRollers tower,
        HopperSubsystem hopper) {

        addCommands(
            // follow auton2 & run intake
            new ParallelDeadlineGroup(
                new PathPlannerAuto("auton2"),
                new IntakeCommand(pivotIntake, rollerIntake)),
            new AutonShooterSequence(flywheel, hood, tower, hopper, pivotIntake)
                .withTimeout(10));
    }
}
