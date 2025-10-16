package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.state.ModelR3;

public class StopDriver implements DriverAdapter {

    private final SwerveDriveSubsystem m_drive;

    public StopDriver(SwerveDriveSubsystem drive) {
        m_drive = drive;
    }

    public void apply(ModelR3 s, Velocity t) {
        m_drive.stop();
    }

    public void reset(ModelR3 p) {
        //
    }

}
