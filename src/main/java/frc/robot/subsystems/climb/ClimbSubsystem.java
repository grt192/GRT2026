package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_MECH_STATE;

public class ClimbSubsystem extends SubsystemBase {
    private StabilizingArmSubsystem m_StabilizingArm;
    private WinchSubsystem m_Winch;

    private CLIMB_MECH_STATE armState = CLIMB_MECH_STATE.FLOATING;
    private CLIMB_MECH_STATE winchState = CLIMB_MECH_STATE.FLOATING;
    private CLIMB_MECH_STATE climbState = CLIMB_MECH_STATE.FLOATING;

    public ClimbSubsystem(CANBus canBusObj) {
        m_StabilizingArm = new StabilizingArmSubsystem(canBusObj);
        m_Winch = new WinchSubsystem(canBusObj);

        armState = m_StabilizingArm.getArmState();
        winchState = m_Winch.getWinchState();
    }

    public void setArmDutyCycle(double dutyCycle) {
        m_StabilizingArm.setMotorDutyCycle(dutyCycle);
    }

    public void setWinchDutyCycle(double dutyCycle) {
        m_Winch.setMotorDutyCycle(dutyCycle);
    }

    public void manualDeployArm() {
        m_StabilizingArm.manualDeployArm();
    }

    public void manualRetractArm() {
        m_StabilizingArm.manualRetractArm();
    }

    public void semiAutoDeployArm() {
        m_StabilizingArm.semiAutoDeployArm();
    }

    public void stopArm() {
        m_StabilizingArm.stop();
    }

    public void manualDeployWinch() {
        m_Winch.manualDeployWinch();
    }

    public void manualHomeWinch() {
        m_Winch.manualHomeWinch();
    }

    public void stopWinch() {
        m_Winch.stop();
    }

    public void setArmTorqueCurrent(Current current) {
        m_StabilizingArm.setTorqueCurrent(current);
    }

    public void setArmPositionSetpoint(Angle setpoint) {
        m_StabilizingArm.setPositionSetpoint(setpoint);
    }

    public void setWinchTorqueCurrent(Current current) {
        m_Winch.setTorqueCurrent(current);
    }

    public Distance getWinchDistance() {
        return m_Winch.getDistance();
    }

    public boolean isWinchAtDistance(Distance target) {
        return m_Winch.isAtDistance(target);
    }

    public boolean isArmAtSetPosition() {
        return m_StabilizingArm.atSetPosition();
    }

    public boolean isArmForwardLimitActive() {
        return m_StabilizingArm.isForwardLimitActive();
    }

    public boolean isArmReverseLimitActive() {
        return m_StabilizingArm.isReverseLimitActive();
    }

    public void updateArmState() {
        armState = m_StabilizingArm.getArmState();
    }

    public void updateWinchState() {
        winchState = m_Winch.getWinchState();
    }

    public CLIMB_MECH_STATE getArmState(boolean refresh) {
        if (refresh) {
            updateArmState();
        }
        return armState;
    }

    public CLIMB_MECH_STATE getArmState() {
        return getArmState(true);
    }

    public CLIMB_MECH_STATE getWinchState(boolean refresh) {
        if (refresh) {
            updateWinchState();
        }
        return winchState;
    }

    public CLIMB_MECH_STATE getWinchState() {
        return getWinchState(true);
    }

    public void updateClimbStates() {
        updateArmState();
        updateWinchState();

        if (armState == CLIMB_MECH_STATE.HOME && winchState == CLIMB_MECH_STATE.HOME) {
            climbState = CLIMB_MECH_STATE.HOME;
        } else if (armState == CLIMB_MECH_STATE.DEPLOYED && winchState == CLIMB_MECH_STATE.DEPLOYED) {
            climbState = CLIMB_MECH_STATE.DEPLOYED;
        } else {
            climbState = CLIMB_MECH_STATE.FLOATING;
        }
    }

    public CLIMB_MECH_STATE getClimbState(boolean refresh) {
        if (refresh) {
            updateClimbStates();
        }
        return climbState;
    }

    public CLIMB_MECH_STATE getClimbState() {
        return getClimbState(true);
    }


    @Override
    public void periodic() {
        updateClimbStates();
        Logger.recordOutput(ClimbConstants.CLIMB_BASE_TABLE + "/climbState", getClimbState(false));
        Logger.recordOutput(ClimbConstants.CLIMB_BASE_TABLE + "/armState", getArmState(false));
        Logger.recordOutput(ClimbConstants.CLIMB_BASE_TABLE + "/winchState", getWinchState(false));
    }
}

