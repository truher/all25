package org.team100.lib.reference.r3;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlR3Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR3Logger;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.trajectory.Trajectory100;

/** Produces references based on a trajectory. */
public class TrajectoryReferenceR3 implements ReferenceR3 {
    private final LoggerFactory m_log;
    private final Trajectory100 m_trajectory;
    private final ModelR3Logger m_log_current;
    private final ControlR3Logger m_log_next;
    private final BooleanLogger m_log_done;
    private final ModelR3Logger m_log_goal;
    private final DoubleLogger m_log_progress;
    private double m_startTimeS;

    public TrajectoryReferenceR3(
            LoggerFactory parent,
            Trajectory100 trajectory) {
        m_log = parent.type(this);
        m_trajectory = trajectory;
        m_log_progress = m_log.doubleLogger(Level.TRACE, "progress");
        m_log_current = m_log.modelR3Logger(Level.TRACE, "current");
        m_log_next = m_log.controlR3Logger(Level.TRACE, "next");
        m_log_done = m_log.booleanLogger(Level.TRACE, "done");
        m_log_goal = m_log.modelR3Logger(Level.TRACE, "goal");
    }

    /** Ignores the measurement, resets the trajectory timer. */
    @Override
    public void initialize(ModelR3 measurement) {
        m_startTimeS = Takt.get();
    }

    @Override
    public ModelR3 current() {
        ModelR3 current = sample(progress()).model();
        m_log_current.log(() -> current);
        return current;
    }

    @Override
    public ControlR3 next() {
        ControlR3 next = sample(progress() + TimedRobot100.LOOP_PERIOD_S);
        m_log_next.log(() -> next);
        return next;
    }

    @Override
    public boolean done() {
        boolean done = m_trajectory.isDone(progress());
        m_log_done.log(() -> done);
        return done;
    }

    @Override
    public ModelR3 goal() {
        ModelR3 goal = ControlR3.fromTimedState(m_trajectory.getLastPoint()).model();
        m_log_goal.log(() -> goal);
        return goal;
    }

    ////////////////////////////////////////////////////

    private double progress() {
        double progress = Takt.get() - m_startTimeS;
        m_log_progress.log(() -> progress);
        return progress;
    }

    private ControlR3 sample(double t) {
        return ControlR3.fromTimedState(m_trajectory.sample(t));
    }
}
