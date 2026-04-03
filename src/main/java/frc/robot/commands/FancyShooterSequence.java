package frc.robot.commands;

import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;
import frc.robot.subsystems.hopper.HopperSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual shooter sequence - no auto-aim.
 * Uses fixed hood position and flywheel RPS from SmashAndShootConstants.
 * Waits for flywheel and hood to reach targets before feeding balls.
 */
public class FancyShooterSequence extends Command {

    private final flywheel fly;
    private final hood hd;
    private final towerRollers tower;
    private final HopperSubsystem hopper;
    private final PivotIntakeSubsystem pivotIntake;
    private final FieldManagementSubsystem fms;
    private final Timer pivotTimer = new Timer();
    private boolean pivotIsIn = true;
    private boolean initialDelayDone = false;
    private Intertable tableThing = new Intertable();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SWERVE_TABLE_NAME");
    StructSubscriber<Pose2d> poseSub = table.getStructTopic("estimatedPose", Pose2d.struct).subscribe(new Pose2d());

    public FancyShooterSequence(
        flywheel fly,
        hood hood,
        towerRollers tower,
        HopperSubsystem hopper,
        PivotIntakeSubsystem pivotIntake,
        FieldManagementSubsystem fms) {
        this.fly = fly;
        this.hd = hood;
        this.tower = tower;
        this.hopper = hopper;
        this.pivotIntake = pivotIntake;
        this.fms = fms;


        addRequirements(fly, hood, tower, hopper, pivotIntake);
    }

    @Override
    public void initialize() {
        double RPS = 0.0;
        double ang = 0.0;

        // Start ramping flywheel and moving hood to position
        if (fms.isRedAlliance()) {
            RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));
            ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));

        } else {
            RPS = tableThing.getRPS(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));
            ang = tableThing.getAngle(poseSub.get().getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));

        }
        fly.shoot(RPS);
        hd.setHoodAngle(ang);
        // Start with pivot out, wait 5 seconds before first toggle
        pivotIsIn = false;
        initialDelayDone = false;
        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
        pivotTimer.restart();
    }

    @Override
    public void execute() {
        // Keep commanding flywheel and hood targets

        // Wait 5 seconds before first pivot up, then toggle every 2 seconds
        if (!initialDelayDone) {
            if (pivotTimer.hasElapsed(SmashAndShootConstants.INITIAL_DELAY_SECONDS)) {
                initialDelayDone = true;
                pivotIsIn = true;
                pivotTimer.restart();
            }
        } else if (pivotTimer.hasElapsed(SmashAndShootConstants.TOGGLE_INTERVAL_SECONDS)) {
            pivotIsIn = !pivotIsIn;
            pivotTimer.restart();
        }
        pivotIntake.setPosition(pivotIsIn ? IntakeConstants.PIVOT_MID_UPPER : IntakeConstants.PIVOT_MID_LOWER);

        // Only feed balls when flywheel is at speed AND hood is at position
        if (/* fly.wantedVel() && hd.wantedAngl() */ true) {
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
        hd.setHoodAngle(0);
        tower.setManualControl(0);
        hopper.setManualControl(0);
        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
    }
}
