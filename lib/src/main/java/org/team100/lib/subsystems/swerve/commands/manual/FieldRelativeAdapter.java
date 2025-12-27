package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;

/** Uses the limited drive subsystem since this is manual driving. */
public class FieldRelativeAdapter implements DriverAdapter {
    private static final boolean DEBUG = false;

    private final SwerveLimiter m_limiter;
    private final VelocitySubsystemSE2 m_drive;
    private final FieldRelativeDriver m_driver;

    public FieldRelativeAdapter(
            SwerveLimiter limiter,
            VelocitySubsystemSE2 drive,
            FieldRelativeDriver driver) {
        m_limiter = limiter;
        m_drive = drive;
        m_driver = driver;
    }

    public void apply(ModelSE2 s, Velocity t) {
        if (DEBUG) {
            System.out.printf("FieldRelativeDriver %s\n", t);
        }
        VelocitySE2 v = m_driver.apply(s, t);
        // scale for driver skill.
        VelocitySE2 scaled = GeometryUtil.scale(v, DriverSkill.level().scale());

        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            scaled = m_limiter.apply(scaled);
        }
        m_drive.setVelocity(scaled);
    }

    public void reset(ModelSE2 p) {
        m_driver.reset(p);
    }

}
