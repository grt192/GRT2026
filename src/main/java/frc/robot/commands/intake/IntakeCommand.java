package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;


public class IntakeCommand extends Command {
    private final PivotIntakeSubsystem pivotIntake;
    private final RollerIntakeSubsystem rollerIntake;

    public IntakeCommand(PivotIntakeSubsystem pivotIntake, RollerIntakeSubsystem rollerIntake) {
        this.pivotIntake = pivotIntake;
        this.rollerIntake = rollerIntake;
        addRequirements(pivotIntake, rollerIntake);
    }

    @Override
    public void initialize() {
        pivotIntake.setPosition(IntakeConstants.PIVOT_IN_POS);
        rollerIntake.runIn();
    }

    @Override
    public void execute() {
        pivotIntake.setPosition(IntakeConstants.PIVOT_IN_POS);
        rollerIntake.runIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rollerIntake.stop();
    }
}
