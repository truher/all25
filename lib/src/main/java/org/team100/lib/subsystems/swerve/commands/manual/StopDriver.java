package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

public class StopDriver implements DriverAdapter {

    private final VelocitySubsystemSE2 m_drive;

    public StopDriver(VelocitySubsystemSE2 drive) {
        m_drive = drive;
    }

    public void apply(ModelSE2 s, Velocity t) {
        m_drive.stop();
    }

    public void reset(ModelSE2 p) {
        //
    }

}
