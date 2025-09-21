package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

public class StopDriver implements DriverAdapter {

    private final SwerveDriveSubsystem m_drive;

    public StopDriver(SwerveDriveSubsystem drive) {
        m_drive = drive;
    }

    public void apply(SwerveModel s, DriverControl.Velocity t) {
        m_drive.stop();
    }

    public void reset(SwerveModel p) {
        //
    }

}
