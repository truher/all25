package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.state.ModelR3;

public class FieldRelativeAdapter implements DriverAdapter {
    private static final boolean DEBUG = false;

    private final SwerveDriveSubsystem m_drive;
    private final FieldRelativeDriver m_driver;

    public FieldRelativeAdapter(SwerveDriveSubsystem drive, FieldRelativeDriver driver) {
        m_drive = drive;
        m_driver = driver;
    }

    public void apply(ModelR3 s, Velocity t) {
        if (DEBUG) {
            System.out.printf("FieldRelativeDriver %s\n", t);
        }
        m_drive.driveInFieldCoords(m_driver.apply(s, t));
    }

    public void reset(ModelR3 p) {
        m_driver.reset(p);
    }

}
