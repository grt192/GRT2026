package frc.robot.subsystems.FMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FieldManagementSubsystem extends SubsystemBase {
    private boolean isRed;
    private boolean connectedToFMS;
    private MatchStatus matchStatus;
    private boolean matchStarted = false;
    private RobotStatus robotStatus;

    private static final double AUTO_END = 20.0;
    private static final double TRANSITION_END = 30.0;
    private static final double ENDGAME_START = 130.0;
    private static final double MATCH_TOTAL = 160.0;
    private static final double TELEOP_SHIFT_DURATION = (ENDGAME_START - TRANSITION_END) / 6.0;

    private boolean redHubActive = true;
    private boolean blueHubActive = true;
    private int currentShift = 0;
    private double timeUntilNextShift = 0.0;
    private boolean redWonAuto = false;

    public FieldManagementSubsystem() {
        isRed = false;
        connectedToFMS = false;
        matchStatus = MatchStatus.NOTSTARTED;
    }

    @Override
    public void periodic() {
        boolean incomingIsRed;
        try {
            incomingIsRed = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
        } catch (Exception e) {
            incomingIsRed = isRed;
        }
        isRed = incomingIsRed;

        connectedToFMS = DriverStation.isFMSAttached();

        double matchTime = DriverStation.getMatchTime();
        double elapsedTime = MATCH_TOTAL - matchTime;

        if (DriverStation.isAutonomous()) {
            matchStatus = MatchStatus.AUTON;
            matchStarted = true;
            redHubActive = true;
            blueHubActive = true;
        } else if (DriverStation.isTeleop()) {
            if (elapsedTime < TRANSITION_END) {
                matchStatus = MatchStatus.TRANSITION;
                redHubActive = true;
                blueHubActive = true;
            } else if (elapsedTime >= ENDGAME_START || matchTime <= 30) {
                matchStatus = MatchStatus.ENDGAME;
                redHubActive = true;
                blueHubActive = true;
            } else {
                matchStatus = MatchStatus.TELEOP;
                double teleopElapsed = elapsedTime - TRANSITION_END;
                currentShift = (int) (teleopElapsed / TELEOP_SHIFT_DURATION);
                double timeInCurrentShift = teleopElapsed % TELEOP_SHIFT_DURATION;
                timeUntilNextShift = TELEOP_SHIFT_DURATION - timeInCurrentShift;

                boolean winnerInactive = (currentShift % 2 == 1);
                if (redWonAuto) {
                    redHubActive = !winnerInactive;
                    blueHubActive = winnerInactive;
                } else {
                    redHubActive = winnerInactive;
                    blueHubActive = !winnerInactive;
                }
            }
        } else if (matchTime == 0 && matchStarted) {
            matchStatus = MatchStatus.ENDED;
        }

        if (DriverStation.isEnabled()) {
            robotStatus = RobotStatus.ENABLED;
        } else if (DriverStation.isEStopped()) {
            robotStatus = RobotStatus.ESTOPPED;
        } else if (DriverStation.isDisabled()) {
            robotStatus = RobotStatus.DISABLED;
        }

        // Log everything through AdvantageKit
        Logger.recordOutput("FMS/IsRedAlliance", isRed);
        Logger.recordOutput("FMS/ConnectedToFMS", connectedToFMS);
        Logger.recordOutput("FMS/MatchStatus", matchStatus.toString());
        Logger.recordOutput("FMS/RobotStatus", robotStatus != null ? robotStatus.toString() : "UNKNOWN");
        Logger.recordOutput("FMS/MatchTimeRemaining", matchTime);
        Logger.recordOutput("FMS/IsAutonomous", DriverStation.isAutonomous());
        Logger.recordOutput("FMS/IsEnabled", DriverStation.isEnabled());
        Logger.recordOutput("FMS/IsEStopped", DriverStation.isEStopped());
        Logger.recordOutput("FMS/RedHubActive", redHubActive);
        Logger.recordOutput("FMS/BlueHubActive", blueHubActive);
        Logger.recordOutput("FMS/CurrentShift", currentShift + 1);
    }

    public boolean isRedAlliance() {
        return isRed;
    }

    public boolean isConnectedToFMS() {
        return connectedToFMS;
    }

    public MatchStatus getMatchStatus() {
        return matchStatus;
    }

    public RobotStatus getRobotStatus() {
        return robotStatus;
    }

    public boolean isRedHubActive() {
        return redHubActive;
    }

    public boolean isBlueHubActive() {
        return blueHubActive;
    }

    public boolean isOurHubActive() {
        return isRed ? redHubActive : blueHubActive;
    }

    public void setAutoWinner(boolean redWon) {
        this.redWonAuto = redWon;
    }

    public int getCurrentShift() {
        return currentShift + 1;
    }
}
