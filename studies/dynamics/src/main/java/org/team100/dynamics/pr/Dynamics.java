package org.team100.dynamics.pr;

public class Dynamics {
    /** Gravity */
    private static final double g = 9.8;
    /** Mass of the moving elevator parts. */
    private final double m1;
    /** Mass of the arm. */
    private final double m2;
    /** Distance from the pivot to the arm center of mass. */
    private final double d;
    /** Moment of inertia of the arm, with respect to the pivot axis. */
    private final double izz;

    public Dynamics(double m1, double m2, double d, double izz) {
        this.m1 = m1;
        this.m2 = m2;
        this.d = d;
        this.izz = izz;
    }

    /**
     * Generalized force (torque or force) to achieve the required
     * velocity and acceleration, and also to oppose gravity.
     */
    public Torque torque(Config q, Velocity v, Acceleration a) {
        double s2 = Math.sin(q.q2());
        double c2 = Math.cos(q.q2());
        double f1 = (m1 + m2) * a.q1ddot()
                - m2 * d * s2 * a.q2ddot()
                - m2 * d * c2 * v.q2dot() * v.q2dot()
                + (m1 + m2) * g;
        double t2 = -m2 * d * s2 * a.q1ddot()
                + (izz + m2 * d * d) * a.q2ddot()
                - d * s2 * m2 * g;
        return new Torque(f1, t2);
    }

}
