package org.team100.lib.profile.incremental;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * WPI Trapezoid profile corrected for gravity.
 * 
 * The actual acceleration constraint on any mechanism is actually motor torque,
 * not acceleration, and for mechanisms that experience variable gravity loads,
 * the available motor torque depends on position.
 * 
 * Position is radians measured from vertical.
 * 
 * This isn't done, and doesn't work well, don't use it.
 * 
 * https://docs.google.com/spreadsheets/d/1MLoYJkBT1qTLl8TmJr8fLQKFw_C2BKwGvfzPcXQUzYk
 */
public class GravityCompensatedProfile implements IncrementalProfile {
    private static final double G = 9.8;

    private final double m_l;

    private final double m_maxV;
    /** Actually available motor torque. */
    private final double m_maxA;

    public GravityCompensatedProfile(double l, double maxV, double maxA) {
        m_l = l;

        m_maxV = maxV;
        m_maxA = maxA;
    }

    @Override
    public Control100 calculate(double dt, Control100 initial, Model100 goal) {
        double i = initial.x();
        double gravityAlpha = G * m_l * Math.sin(i);
        double g = goal.x();

        // Maybe we're pushing up, so gravity is hurting?
        Control100 weak = calc(dt, initial, goal, m_maxA - gravityAlpha);
        if (weak.a() < 0) {
            // Actually pushing up, so this is the right answer
            return weak;
        }
        // Maybe we're pushing down, so gravity is helping?
        Control100 strong = calc(dt, initial, goal, m_maxA + gravityAlpha);
        if (strong.a() > 0) {
            // Actually pushing down, so this is the right answer
            return strong;
        }
        // If we get here, we're where the soft profile would like to start
        // slowing down, but the firm one would not. It's not obvious what to do.
        return weak;

    }

    /** Try with the specified max accel */
    private Control100 calc(double dt, Control100 initial, Model100 goal, double availableAccel) {
        if (availableAccel < 0)
            throw new IllegalStateException("motor weaker than g");
        Constraints m_constraints = new Constraints(m_maxV, availableAccel);
        TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);
        State result = m_profile.calculate(dt, new State(initial.x(), initial.v()), new State(goal.x(), goal.v()));
        // WPI State doesn't have accel, so we calculate it.
        double accel = (result.velocity - initial.v()) / dt;
        return new Control100(result.position, result.velocity, accel);
    }

    @Override
    public IncrementalProfile scale(double s) {
        return new GravityCompensatedProfile(m_l, m_maxV, s * m_maxA);
    }

}
