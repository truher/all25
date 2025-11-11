package org.team100.lib.dynamics.prr;

public class PRRDynamics {
    /** Gravity */
    private static final double g = 9.8;

    /** Mass of the moving elevator parts. */
    private final double m1;
    /** Mass of the first revolute link. */
    private final double m2;
    /** Mass of the second revolute link. */
    private final double m3;

    // (The l1 and l3 lengths don't matter.)
    /** Length of link 2 */
    private final double l2;

    // (The lc1 length doesn't matter.)
    /** Distance from q2 to the first revolute link center of mass. */
    private final double lc2;
    /** Distance from q3 to the second revolute link center of mass. */
    private final double lc3;

    /** Moment of inertia of the first revolute link. */
    private final double izz2;
    /** Moment of inertia of the second revolute link. */
    private final double izz3;

    public PRRDynamics(
            double m1, double m2, double m3,
            double l2,
            double lc2, double lc3,
            double izz2, double izz3) {
        this.m1 = m1;
        this.m2 = m2;
        this.m3 = m3;
        this.l2 = l2;
        this.lc2 = lc2;
        this.lc3 = lc3;
        this.izz2 = izz2;
        this.izz3 = izz3;
    }

    /**
     * Generalized force (torque or force) to achieve the required
     * velocity and acceleration, and also to oppose gravity.
     */
    public PRRTorque torque(PRRConfig q, PRRVelocity v, PRRAcceleration a) {
        double s2 = Math.sin(q.q2());
        double c2 = Math.cos(q.q2());
        double s23 = Math.sin(q.q2() + q.q3());
        double c23 = Math.cos(q.q2() + q.q3());
        double s3 = Math.sin(q.q3());
        double c3 = Math.cos(q.q3());

        double f1 = (m1 + m2 + m3) * a.q1ddot()
                + (-m2 * lc2 * s2 - m3 * l2 * s2 - m3 * lc3 * s23) * a.q2ddot()
                - m3 * lc3 * s23 * a.q3ddot()
                + (-m2 * lc2 * c2 * v.q2dot() - m3 * l2 * c2 * v.q2dot() - m3 * lc3 * c23 * v.q2dot()
                        - m3 * lc3 * c23 * v.q3dot()) * v.q2dot()
                + (-m3 * lc3 * c23 * v.q2dot() - m3 * lc3 * c23 * v.q3dot()) * v.q3dot()
                + (m1 + m2 + m3) * g;

        double t2 = (-m2 * lc2 * s2 - m3 * l2 * s2 - m3 * lc3 * s23) * a.q1ddot()
                + (m2 * lc2 * lc2 + izz2 + m3 * l2 * l2 + m3 * 2 * l2 * lc3 * c3 + m3 * lc3 * lc3 + izz3) * a.q2ddot()
                + (m3 * lc3 * lc3 + m3 * l2 * lc3 * c3 + izz3) * a.q3ddot()
                + (-m3 * l2 * lc3 * s3 * v.q3dot()) * v.q2dot()
                + (-m3 * l2 * lc3 * s3 * v.q2dot() - m3 * l2 * lc3 * s3 * v.q3dot()) * v.q3dot()
                + (-lc2 * s2 * m2 * g - l2 * s2 * m3 * g - lc2 * s23 * m3 * g);

        double t3 = (-m3 * lc3 * s23) * a.q1ddot()
                + (m3 * lc3 * lc3 + m3 * l2 * lc3 * c3 + izz3) * a.q2ddot()
                + (m3 * lc3 * lc3 + izz3) * a.q3ddot()
                + m3 * l2 * lc3 * s3 * v.q2dot() * v.q2dot()
                - lc3 * s23 * m3 * g;

        return new PRRTorque(f1, t2, t3);
    }

}
