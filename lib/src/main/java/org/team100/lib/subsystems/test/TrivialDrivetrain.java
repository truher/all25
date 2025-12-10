package org.team100.lib.subsystems.test;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Executes desired velocity exactly.
 */
public class TrivialDrivetrain implements VelocitySubsystemR3 {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final ObjectCache<ModelR3> m_stateCache;
    /** setpoint for the next timestep. used only by update(). */
    private GlobalVelocityR3 m_setpoint;
    private ModelR3 m_state;

    public TrivialDrivetrain(Pose2d initial) {
        m_setpoint = new GlobalVelocityR3(0, 0, 0);
        m_state = new ModelR3(initial);
        m_stateCache = Cache.of(this::update);
    }

    @Override
    public ModelR3 getState() {
        return m_stateCache.get();
    }

    @Override
    public void stop() {
        m_setpoint = new GlobalVelocityR3(0, 0, 0);
    }

    @Override
    public void setVelocity(GlobalVelocityR3 setpoint) {
        m_setpoint = setpoint;
    }

    private ModelR3 update() {
        m_state = new ModelR3(
                m_setpoint.integrate(m_state.pose(), DT),
                m_setpoint);
        return m_state;
    }

}
