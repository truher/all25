package org.team100.lib.motion.drivetrain;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ModelR3;

public class MockDrive implements DriveSubsystemInterface {
    public GlobalVelocityR3 m_setpoint;
    public GlobalVelocityR3 m_recentSetpoint;
    public ModelR3 m_state;

    @Override
    public void driveInFieldCoords(GlobalVelocityR3 setpoint) {
        m_setpoint = setpoint;
        m_recentSetpoint = setpoint;
    }

    @Override
    public ModelR3 getState() {
        return m_state;
    }

    @Override
    public void stop() {
        // do nothing
    }

    @Override
    public void driveInFieldCoordsVerbatim(GlobalVelocityR3 setpoint) {
        driveInFieldCoords(setpoint);
    }

    @Override
    public void resetLimiter() {
        //
    }

}
