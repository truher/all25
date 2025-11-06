package org.team100.lib.reference.r1;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.SetpointsR1Logger;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Model100;

/**
 * Extracts current and next references from a timed profile.
 */
public class TimedProfileReferenceR1 implements ProfileReferenceR1 {
    private final SetpointsR1Logger m_log_setpoints;
    private final BooleanLogger m_log_done;
    private final TimedProfile m_profile;
    private Model100 m_goal;
    private double m_startTimeS;

    public TimedProfileReferenceR1(LoggerFactory parent, TimedProfile profile) {
        LoggerFactory log = parent.type(this);
        m_log_setpoints = log.setpointsR1Logger(Level.TRACE, "setpoints");
        m_log_done = log.booleanLogger(Level.TRACE, "done");
        m_profile = profile;
    }

    @Override
    public void setGoal(Model100 goal) {
        m_goal = goal;
    }

    @Override
    public void init(Model100 measurement) {
        if (m_goal == null)
            throw new IllegalStateException("goal must be set");
        m_startTimeS = Takt.get();
        m_profile.init(measurement.control(), m_goal);
    }

    @Override
    public SetpointsR1 get() {
        double progress = progress();
        SetpointsR1 setpoints = new SetpointsR1(
                m_profile.sample(progress),
                m_profile.sample(progress + TimedRobot100.LOOP_PERIOD_S));
        m_log_setpoints.log(() -> setpoints);
        return setpoints;
    }

    @Override
    public boolean profileDone() {
        boolean done = progress() >= duration();
        m_log_done.log(() -> done);
        return done;
    }

    //////////////////////////////////////////////////////////////////////////////////////

    private double duration() {
        return m_profile.duration();
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

}
