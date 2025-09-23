package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.util.Util;

public class FieldRelativeAdapter implements DriverAdapter {
    private static final boolean DEBUG = false;

    private final SwerveDriveSubsystem m_drive;
    private final FieldRelativeDriver m_driver;

    public FieldRelativeAdapter(SwerveDriveSubsystem drive, FieldRelativeDriver driver) {
        m_drive = drive;
        m_driver = driver;
    }

    public void apply(SwerveModel s, DriverControl.Velocity t) {
        if (DEBUG)
            Util.printf("FieldRelativeDriver %s\n", t);
        m_drive.driveInFieldCoords(m_driver.apply(s, t));
    }

    public void reset(SwerveModel p) {
        m_driver.reset(p);
    }

}
