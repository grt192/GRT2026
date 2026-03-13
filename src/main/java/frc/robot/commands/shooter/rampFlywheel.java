package frc.robot.commands.shooter;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.subsystems.shooter.Intertable;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.math.geometry.Pose2d;


public class rampFlywheel extends Command {

    private final flywheel fly;
    private final SwerveSubsystem swerve;
    private final FieldManagementSubsystem fms;
    private final Intertable referenceTable = Intertable.getInstance();

    public rampFlywheel(flywheel h, FieldManagementSubsystem fms) {
        this.fly = h;
        this.swerve = swerve;
        this.fms = fms;
        addRequirements(fly);

        NetworkTable learnerTable = NetworkTableInstance.getDefault().getTable("ShooterLearner");
        offsetEntry = learnerTable.getEntry("offset");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double RPS = 0;
        boolean redTeam = fms.isRedAlliance();
        Pose2d pose = swerve.getRobotPosition();

        if (redTeam) {
            if (pose.getX() > AlignConstants.RED_WALL_X) {
                RPS = referenceTable.getRPS(pose.getTranslation().getDistance(AlignConstants.RED_HUB_TRANS));
            } else {
                if (pose.getY() > AlignConstants.HUB_Y) {
                    RPS = referenceTable.getRPS(pose.getTranslation().getDistance(AlignConstants.RED_AIM_TOP));
                } else {
                    RPS = referenceTable.getRPS(pose.getTranslation().getDistance(AlignConstants.RED_AIM_BOTTOM));
                }
            }

        } else {
            if (pose.getX() < AlignConstants.BLUE_WALL_X) {
                RPS = referenceTable.getRPS(pose.getTranslation().getDistance(AlignConstants.BLUE_HUB_TRANS));
            } else {
                if (pose.get().getY() > AlignConstants.HUB_Y) {
                    RPS = referenceTable.getRPS(pose.getTranslation().getDistance(AlignConstants.BLUE_AIM_TOP));
                } else {
                    RPS = referenceTable.getRPS(pose.getTranslation().getDistance(AlignConstants.BLUE_AIM_BOTTOM));
                }
            }
        }

        fly.shoot(RPS + offsetEntry.getDouble(0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        fly.dontShoot();
    }

}
