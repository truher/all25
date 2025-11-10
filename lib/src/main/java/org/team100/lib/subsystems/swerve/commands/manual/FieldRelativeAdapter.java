package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

/** Uses the limited drive subsystem since this is manual driving. */
public class FieldRelativeAdapter implements DriverAdapter {
    private static final boolean DEBUG = false;

    private final SwerveLimiter m_limiter;
    private final VelocitySubsystemR3 m_drive;
    private final FieldRelativeDriver m_driver;

    public FieldRelativeAdapter(
            SwerveLimiter limiter,
            VelocitySubsystemR3 drive,
            FieldRelativeDriver driver) {
        m_limiter = limiter;
        m_drive = drive;
        m_driver = driver;
    }

    public void apply(ModelR3 s, Velocity t) {
        if (DEBUG) {
            System.out.printf("FieldRelativeDriver %s\n", t);
        }
        GlobalVelocityR3 v = m_driver.apply(s, t);
        // scale for driver skill.
        GlobalVelocityR3 scaled = GeometryUtil.scale(v, DriverSkill.level().scale());

        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            scaled = m_limiter.apply(scaled);
        }
        m_drive.setVelocity(scaled);
    }

    public void reset(ModelR3 p) {
        m_driver.reset(p);
    }

}
