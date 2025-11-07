package org.team100.lib.commands.swerve.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

public class StopDriver implements DriverAdapter {

    private final VelocitySubsystemR3 m_drive;

    public StopDriver(VelocitySubsystemR3 drive) {
        m_drive = drive;
    }

    public void apply(ModelR3 s, Velocity t) {
        m_drive.stop();
    }

    public void reset(ModelR3 p) {
        //
    }

}
