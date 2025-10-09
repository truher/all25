package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

public class MockDrive implements DriveSubsystemInterface {
    public GlobalSe2Velocity m_setpoint;
    public GlobalSe2Velocity m_recentSetpoint;
    public SwerveModel m_state;

    @Override
    public void driveInFieldCoords(GlobalSe2Velocity setpoint) {
        m_setpoint = setpoint;
        m_recentSetpoint = setpoint;
    }

    @Override
    public SwerveModel getState() {
        return m_state;
    }

    @Override
    public void stop() {
        // do nothing
    }

    @Override
    public void driveInFieldCoordsVerbatim(GlobalSe2Velocity setpoint) {
        driveInFieldCoords(setpoint);
    }

    @Override
    public void resetLimiter() {
        //
    }

}
