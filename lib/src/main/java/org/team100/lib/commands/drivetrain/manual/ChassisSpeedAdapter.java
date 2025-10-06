package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.util.Util;

public class ChassisSpeedAdapter implements DriverAdapter {
    private static final boolean DEBUG = false;

    private final SwerveDriveSubsystem m_drive;
    private final ChassisSpeedDriver m_driver;

    public ChassisSpeedAdapter(SwerveDriveSubsystem drive, ChassisSpeedDriver driver) {
        m_drive = drive;
        m_driver = driver;
    }

    @Override
    public void apply(SwerveModel s, Velocity t) {
        if (DEBUG)
            Util.printf("ChassisSpeedDriver %s\n", t);
        m_drive.setChassisSpeeds(m_driver.apply(s, t));
    }

    @Override
    public void reset(SwerveModel p) {
        m_driver.reset(p);
    }
}
