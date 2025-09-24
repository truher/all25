package org.team100.lib.reference;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.trajectory.Trajectory100;

/** Produces references based on a trajectory. */
public class TrajectoryReference implements SwerveReference {
    private final Trajectory100 m_trajectory;
    private double m_startTimeS;

    public TrajectoryReference(Trajectory100 trajectory) {
        m_trajectory = trajectory;
    }

    /** Ignores the measurement, resets the trajectory timer. */
    @Override
    public void initialize(SwerveModel measurement) {
        m_startTimeS = Takt.get();
    }

    @Override
    public SwerveModel current() {
        return sample(progress()).model();
    }

    @Override
    public SwerveControl next() {
        return sample(progress() + TimedRobot100.LOOP_PERIOD_S);
    }

    @Override
    public boolean done() {
        return m_trajectory.isDone(progress());
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

    private SwerveControl sample(double t) {
        return SwerveControl.fromTimedPose(m_trajectory.sample(t));
    }

}
