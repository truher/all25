package org.team100.lib.trajectory.path.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlotter;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.CapsizeAccelerationConstraint;
import org.team100.lib.trajectory.timing.ScheduleGenerator;
import org.team100.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class HolonomicSplineTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testCourse() {
        Rotation2d course = new Rotation2d(Math.PI / 4);
        Translation2d t = new Translation2d(1, 0).rotateBy(course);
        assertEquals(0.707, t.getX(), DELTA);
        assertEquals(0.707, t.getY(), DELTA);
    }

    @Test
    void testLinear() {
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        DirectionSE2.TO_X, 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        DirectionSE2.TO_X, 1));

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));

        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), DELTA);
        t = s.getPoint(1);
        assertEquals(1, t.getX(), DELTA);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
        p = s.getPose2dWithMotion(1);
        assertEquals(1, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
    }

    @Test
    void testLinear2() {
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        DirectionSE2.TO_X, 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(2, 0),
                                new Rotation2d()),
                        DirectionSE2.TO_X, 1));

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));

        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), DELTA);
        t = s.getPoint(1);
        assertEquals(2, t.getX(), DELTA);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
        p = s.getPose2dWithMotion(1);
        assertEquals(2, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
    }

    @Test
    void testRotationSoft() {
        // move ahead 1m while rotation 1 rad to the left
        // this has no rotation at the ends.
        // the rotation rate is zero at the ends and much higher in the middle.
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        DirectionSE2.TO_X, 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        DirectionSE2.TO_X, 1.2));

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));

        if (DEBUG)
            s.printSamples();

        // now that the magic numbers apply to the rotational scaling, the heading rate
        // is constant.
        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), DELTA);
        t = s.getPoint(1);
        assertEquals(1, t.getX(), DELTA);

        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        // initial rotation rate is zero
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(0.5);
        assertEquals(0.5, p.getPose().translation().getX(), DELTA);
        assertEquals(0.5, p.getPose().heading().getRadians(), DELTA);
        // high rotation rate in the middle
        assertEquals(4.807, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(1);
        assertEquals(1, p.getPose().translation().getX(), DELTA);
        assertEquals(1, p.getPose().heading().getRadians(), DELTA);
        // rotation rate is zero at the end
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);

    }

    @Test
    void testRotationFast() {
        // move ahead 1m while rotation 1 rad to the left
        // this has lots of rotation at the ends
        // the "spatial" rotation rate is constant, i.e.
        // rotation and translation speed are proportional.
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 1), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 1), 1));

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));

        s.printSamples();

        // now that the magic numbers apply to the rotational scaling, the heading rate
        // is constant.
        Translation2d t = s.getPoint(0);
        assertEquals(0, t.getX(), DELTA);
        t = s.getPoint(1);
        assertEquals(1, t.getX(), DELTA);

        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(1, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(0.5);
        assertEquals(0.5, p.getPose().translation().getX(), DELTA);
        assertEquals(0.5, p.getPose().heading().getRadians(), DELTA);
        assertEquals(1, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(1);
        assertEquals(1, p.getPose().translation().getX(), DELTA);
        assertEquals(1, p.getPose().heading().getRadians(), DELTA);
        assertEquals(1, p.getHeadingRateRad_M(), DELTA);

    }

    @Test
    void testRotation2() {
        // Make sure the rotation goes over +/- pi
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d(2.5)),
                        DirectionSE2.TO_X, 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(-2.5)),
                        DirectionSE2.TO_X, 1));
        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));
    }

    /** Turning in place splines work. */
    @Test
    void spin() {
        double scale = 0.9;
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 0, 1), scale),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kCCW_90deg),
                        new DirectionSE2(0, 0, 1), scale));

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(s0);

        // before
        assertTrue(verifyC1(splines));
        // spline joints are C2 smooth (i.e. same curvature)
        assertTrue(verifyC2(splines));

        // optimize and compare
        TrajectoryPlotter.plot(splines, 0.1, 1);
    }

    @Test
    void testCircle() {
        // four splines that make a circle should have nice even curvature and velocity
        // throughout. The circle is centered at zero, the heading always points there.
        //
        // This now uses the specification of SE(2) "course" which includes rotation.
        //
        // I fiddled with the scale to make a pretty good circle.
        double scale = 0.9;
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.k180deg),
                        new DirectionSE2(0, 1, 1), scale),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 1),
                                Rotation2d.kCW_90deg),
                        new DirectionSE2(-1, 0, 1), scale));
        HolonomicSpline s1 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 1),
                                Rotation2d.kCW_90deg),
                        new DirectionSE2(-1, 0, 1), scale),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(-1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, -1, 1), scale));
        HolonomicSpline s2 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(-1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, -1, 1), scale),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, -1),
                                Rotation2d.kCCW_90deg),
                        new DirectionSE2(1, 0, 1), scale));
        HolonomicSpline s3 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, -1),
                                Rotation2d.kCCW_90deg),
                        new DirectionSE2(1, 0, 1), scale),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.k180deg),
                        new DirectionSE2(0, 1, 1), scale));
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(s0);
        splines.add(s1);
        splines.add(s2);
        splines.add(s3);

        // before
        assertTrue(verifyC1(splines));
        // spline joints are C2 smooth (i.e. same curvature)
        assertTrue(verifyC2(splines));

        checkCircle(splines, 0.011, 0.005);

        // optimize and compare
        TrajectoryPlotter.plot(splines, 0.1, 1);

    }

    private void checkCircle(List<HolonomicSpline> splines, double rangeError, double azimuthError) {
        double actualRangeError = 0;
        double actualAzimuthError = 0;
        for (HolonomicSpline s : splines) {
            for (double j = 0; j < 0.99; j += 0.1) {
                Pose2d p = s.getPose2d(j);
                // the position should be on the circle
                double range = p.getTranslation().getNorm();
                actualRangeError = Math.max(actualRangeError, Math.abs(1.0 - range));

                // the heading should point to the origin all the time.
                Rotation2d angleFromOrigin = p.getTranslation().unaryMinus().getAngle();
                Rotation2d error = angleFromOrigin.minus(p.getRotation());
                // there's about 2 degrees of error here because the spline is not quite a
                // circle.
                // 3/10/25 i made generation coarser so it's less accurate.
                actualAzimuthError = Math.max(actualAzimuthError, Math.abs(error.getRadians()));
            }
        }
        assertEquals(0, actualRangeError, rangeError);
        assertEquals(0, actualAzimuthError, azimuthError);

    }

    @Test
    void testLine() {
        // turn a bit to the left
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        DirectionSE2.TO_X, 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        DirectionSE2.TO_X, 1));
        // turn much more to the left
        HolonomicSpline s1 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        DirectionSE2.TO_X, 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(2, 0),
                                Rotation2d.k180deg),
                        DirectionSE2.TO_X, 1));
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(s0);
        splines.add(s1);

        TrajectoryPlotter.plot(splines, 0.1, 1);

        // spline joints are not C1 smooth
        assertFalse(verifyC1(splines));
        // spline joints are C2 smooth (i.e. same curvature)
        assertTrue(verifyC2(splines));
    }

    /**
     * A kinda-realistic test path:
     * 
     * * start facing towards the driver
     * * back up
     * * rotate towards +y, also drive towards +y
     * 
     * Does optimization really help here?
     */
    @Test
    void testPath0() {
        double scale = 0.7;
        WaypointSE2 p0 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d(-1, 0)),
                new DirectionSE2(1, 0, 0), scale);
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0.707, 0.293),
                        new Rotation2d(-1, 1)),
                new DirectionSE2(1, 1, -1), scale);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(1, 1),
                        new Rotation2d(0, 1)),
                new DirectionSE2(0, 1, 0), scale);

        if (DEBUG)
            System.out.println("s01");
        HolonomicSpline s01 = new HolonomicSpline(p0, p1);
        if (DEBUG)
            System.out.println("s12");
        HolonomicSpline s12 = new HolonomicSpline(p1, p2);
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(s01);
        splines.add(s12);

        // before
        assertTrue(verifyC1(splines));
        assertTrue(verifyC2(splines));

        TrajectoryPlotter.plot(splines, 0.1, 1);

        // after
        assertTrue(verifyC1(splines));
        assertTrue(verifyC2(splines));
    }

    @Test
    void testMismatchedXYDerivatives() {
        // corners seem to be allowed, but yield un-traversable trajectories
        // with infinite accelerations at the corners, so they should be prohibited.

        // this goes straight ahead to (1,0)
        // derivatives point straight ahead
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        DirectionSE2.TO_X, 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        DirectionSE2.TO_X, 1));
        // this is a sharp turn to the left
        // derivatives point to the left
        HolonomicSpline s1 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        DirectionSE2.TO_Y, 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                Rotation2d.kZero),
                        DirectionSE2.TO_Y, 1));
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(s0);
        splines.add(s1);

        TrajectoryPlotter.plot(splines, 0.1, 1);

        for (HolonomicSpline s : splines) {
            if (DEBUG)
                System.out.printf("spline %s\n", s);
        }

        // spline joints are C1 smooth (i.e. same slope)
        assertFalse(verifyC1(splines));
        // spline joints are C2 smooth (i.e. same curvature)
        assertTrue(verifyC2(splines));

        Path100 path = new Path100(PathFactory.parameterizeSplines(splines, 0.05, 0.05, 0.05));
        if (DEBUG)
            System.out.printf("path %s\n", path);
        List<TimingConstraint> constraints = new ArrayList<>();
        ScheduleGenerator scheduleGenerator = new ScheduleGenerator(constraints);
        Trajectory100 trajectory = scheduleGenerator.timeParameterizeTrajectory(path,
                0.05, 0, 0);

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("plot", trajectory);

        if (DEBUG)
            System.out.printf("trajectory %s\n", trajectory);

    }

    @Test
    void testEntryVelocity() {

        // radius is 1 m.
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, -1),
                                Rotation2d.kZero),
                        DirectionSE2.TO_X, 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        DirectionSE2.TO_Y, 1.2));
        if (DEBUG) {
            for (double t = 0; t < 1; t += 0.03) {
                System.out.printf("%5.3f %5.3f\n", s0.x(t), s0.y(t));
            }
        }

        List<HolonomicSpline> splines = List.of(s0);

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("splines", splines);

        List<Pose2dWithMotion> motion = PathFactory.parameterizeSplines(splines, 0.05, 0.05, 0.05);
        if (DEBUG) {
            for (Pose2dWithMotion p : motion) {
                System.out.printf("%5.3f %5.3f\n", p.getPose().translation().getX(), p.getPose().translation().getY());
            }
        }
        Path100 path = new Path100(motion);
        if (DEBUG) {
            for (int i = 0; i < path.length(); ++i) {
                System.out.printf("%5.3f %5.3f\n",
                        path.getPoint(i).getPose().translation().getX(),
                        path.getPoint(i).getPose().translation().getY());
            }
        }

        // if we enter a circle at the capsize velocity, we should continue at that same
        // speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest(logger);
        // centripetal accel is 8.166 m/s^2
        assertEquals(8.166666, limits.getMaxCapsizeAccelM_S2(), 1e-6);
        List<TimingConstraint> constraints = List.of(
                new CapsizeAccelerationConstraint(logger, limits, 1.0));
        ScheduleGenerator scheduleGenerator = new ScheduleGenerator(constraints);
        // speed
        // a = v^2/r so v = sqrt(ar) = 2.858
        Trajectory100 trajectory = scheduleGenerator.timeParameterizeTrajectory(path,
                0.05, 2.858, 2.858);

        plotter.plot("plot", trajectory);

        if (DEBUG)
            System.out.printf("trajectory %s\n", trajectory);
    }

    /**
     * True if adjacent spline endpoints have (nearly) identical derivative terms.
     */
    static boolean verifyC1(List<HolonomicSpline> splines) {
        if (splines.size() < 2)
            return true;
        for (int i = 0; i < splines.size() - 1; ++i) {
            HolonomicSpline s0 = splines.get(i);
            HolonomicSpline s1 = splines.get(i + 1);
            if (!MathUtil.isNear(s0.dx(1), s1.dx(0), 1e-6)) {
                if (DEBUG) {
                    System.out.printf("bad x C1 %f %f\n", s0.dx(1), s1.dx(0));
                }
                return false;
            }
            if (!MathUtil.isNear(s0.dy(1), s1.dy(0), 1e-6)) {
                if (DEBUG) {
                    System.out.printf("bad y C1 %f %f\n", s0.dy(1), s1.dy(0));
                }
                return false;
            }
            if (!MathUtil.isNear(s0.dtheta(1), s1.dtheta(0), 1e-6)) {
                if (DEBUG) {
                    System.out.printf("bad theta C1 %f %f\n", s0.dtheta(1), s1.dtheta(0));
                }
                return false;
            }
        }
        return true;
    }

    /**
     * True if adjacent spline endpoints have (nearly) identical second-derivative
     * terms.
     */
    static boolean verifyC2(List<HolonomicSpline> splines) {
        if (splines.size() < 2)
            return true;
        for (int i = 0; i < splines.size() - 1; ++i) {
            HolonomicSpline s0 = splines.get(i);
            HolonomicSpline s1 = splines.get(i + 1);
            if (!MathUtil.isNear(s0.ddx(1), s1.ddx(0), 1e-6)) {
                if (DEBUG) {
                    System.out.printf("bad x C2 %f %f\n", s0.ddx(1), s1.ddx(0));
                }
                return false;
            }
            if (!MathUtil.isNear(s0.ddy(1), s1.ddy(0), 1e-6)) {
                if (DEBUG) {
                    System.out.printf("bad y C2 %f %f\n", s0.ddy(1), s1.ddy(0));
                }
                return false;
            }
            if (!MathUtil.isNear(s0.ddtheta(1), s1.ddtheta(0), 1e-6)) {
                if (DEBUG) {
                    System.out.printf("bad theta C2 %f %f\n", s0.ddtheta(1), s1.ddtheta(0));
                }
                return false;
            }
        }
        return true;
    }
}
