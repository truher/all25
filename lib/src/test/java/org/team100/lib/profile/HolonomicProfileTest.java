package org.team100.lib.profile;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.motion.drivetrain.SwerveControl;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class HolonomicProfileTest {
    private static final boolean PRINT = true;

    /**
     * This uses the TrapezoidProfile100, which is the Team100 state-space thing.
     */
    @Test
    void test2d() {
        HolonomicProfile hp = HolonomicProfile.trapezoidal(1, 1, 0.01, 1, 1, 0.01);
        SwerveModel i = new SwerveModel();
        SwerveModel g = new SwerveModel(new Pose2d(1, 5, Rotation2d.kZero));
        hp.solve(i, g);
        SwerveControl s = i.control();
        for (double t = 0; t < 10; t += 0.02) {
            s = hp.calculate(s.model(), g);
            if (PRINT)
                Util.printf("%.2f %.3f %.3f\n", t, s.x().x(), s.y().x());
        }
    }

    /**
     * Uses combined trapezoid and exponential, modeling the current limiter. the
     * main effect here is that decel is very fast.
     */
    @Test
    void test2dExp() {
        HolonomicProfile hp = HolonomicProfile.currentLimitedExponential(1, 1, 2, 1, 1, 2);
        SwerveModel i = new SwerveModel();
        SwerveModel g = new SwerveModel(new Pose2d(1, 5, Rotation2d.kZero));
        hp.solve(i, g);
        SwerveControl s = i.control();
        for (double t = 0; t < 10; t += 0.02) {
            s = hp.calculate(s.model(), g);
            if (PRINT)
                Util.printf("%.2f %.3f %.3f\n", t, s.x().x(), s.y().x());
        }
    }

    @Test
    void test2dWithEntrySpeed() {
        HolonomicProfile hp = HolonomicProfile.trapezoidal(1, 1, 0.01, 1, 1, 0.01);
        SwerveModel i = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        SwerveModel g = new SwerveModel(new Pose2d(0, 1, Rotation2d.kZero));
        hp.solve(i, g);
        SwerveControl s = i.control();
        for (double t = 0; t < 10; t += 0.02) {
            s = hp.calculate(s.model(), g);
            if (PRINT)
                Util.printf("%.2f %.3f %.3f\n", t, s.x().x(), s.y().x());
        }
    }

    /**
     * On my desktop, the solve() method takes about 1 microsecond, so it seems
     * ok to not worry about how long it takes.
     * 
     * With the simulation approach to ETA with full-scale DT this takes 8 us.
     * With the 10x coarser DT it is 1.8 us.
     * 
     * Removing the ETA calculation from the TrapezoidProfile100, i.e. using
     * simulation on it, makes this much slower, 0.1 ms. Since this happens
     * once at the start of the profile (for coordination), that's fine.
     * 
     * The SOLVE_DT constant in HolonomicProfile affects performance ~linearly.
     */
    @Test
    void testSolvePerformance() {
        HolonomicProfile hp = HolonomicProfile.trapezoidal(1, 1, 0.01, 1, 1, 0.01);
        SwerveModel i = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        SwerveModel g = new SwerveModel(new Pose2d(0, 1, Rotation2d.kZero));
        int N = 10000;
        double t0 = Takt.actual();
        for (int ii = 0; ii < N; ++ii) {
            hp.solve(i, g);
        }
        double t1 = Takt.actual();
        if (PRINT)
            Util.printf("duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
        if (PRINT)
            Util.printf("per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);
    }
}
