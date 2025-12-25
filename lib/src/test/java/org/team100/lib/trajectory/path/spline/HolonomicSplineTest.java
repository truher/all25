package org.team100.lib.trajectory.path.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.WaypointSE2;
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
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.ScheduleGenerator;
import org.team100.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class HolonomicSplineTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testCurvature() {
        // straight line, zero curvature.
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1));
        assertEquals(0, s.getCurvature(0.5), DELTA);

        // left turn
        s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1));
        assertEquals(0.950, s.getCurvature(0.5), DELTA);

        // rotation in place yields zero curvature since there is no x/y motion, so
        // curvature has no meaning.
        s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(0, 0, 1), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d(1)),
                        new DirectionSE2(0, 0, 1), 1));
        assertEquals(0, s.getCurvature(0.5), DELTA);
    }

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
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1));

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));

        Translation2d t = s.getPose2d(0).getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.getPose2d(1).getTranslation();
        assertEquals(1, t.getX(), DELTA);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.getPose().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
        p = s.getPose2dWithMotion(1);
        assertEquals(1, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.getPose().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
    }

    @Test
    void testLinear2() {
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(2, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1));

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));

        Translation2d t = s.getPose2d(0).getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.getPose2d(1).getTranslation();
        assertEquals(2, t.getX(), DELTA);
        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.getPose().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
        p = s.getPose2dWithMotion(1);
        assertEquals(2, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.getPose().pose().getRotation().getRadians(), DELTA);
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
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 0), 1.2));

        TrajectoryPlotter plotter = new TrajectoryPlotter(0.1);
        plotter.plot("rotation", List.of(s));

        // now that the magic numbers apply to the rotational scaling, the heading rate
        // is constant.
        Translation2d t = s.getPose2d(0).getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.getPose2d(1).getTranslation();
        assertEquals(1, t.getX(), DELTA);

        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.getPose().pose().getRotation().getRadians(), DELTA);
        // initial rotation rate is zero
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(0.5);
        assertEquals(0.5, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.5, p.getPose().pose().getRotation().getRadians(), DELTA);
        // high rotation rate in the middle
        assertEquals(0.915, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(1);
        assertEquals(1, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(1, p.getPose().pose().getRotation().getRadians(), DELTA);
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

        // now that the magic numbers apply to the rotational scaling, the heading rate
        // is constant.
        Translation2d t = s.getPose2d(0).getTranslation();
        assertEquals(0, t.getX(), DELTA);
        t = s.getPose2d(1).getTranslation();
        assertEquals(1, t.getX(), DELTA);

        Pose2dWithMotion p = s.getPose2dWithMotion(0);
        assertEquals(0, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.getPose().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.707, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(0.5);
        assertEquals(0.5, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0.5, p.getPose().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.707, p.getHeadingRateRad_M(), DELTA);

        p = s.getPose2dWithMotion(1);
        assertEquals(1, p.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(1, p.getPose().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.707, p.getHeadingRateRad_M(), DELTA);

    }

    @Test
    void testRotation2() {
        // Make sure the rotation goes over +/- pi
        HolonomicSpline s = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d(2.5)),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(-2.5)),
                        new DirectionSE2(1, 0, 0), 1));
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
        TrajectoryPlotter.plot(splines, 0.1);
    }

    @Test
    void testCircle() {
        // four splines that make a circle should have nice even curvature and velocity
        // throughout. The circle is centered at zero, the heading always points there.
        double scale = 1.3;
        WaypointSE2 p0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(new Translation2d(-1, 0), Rotation2d.kZero),
                new DirectionSE2(0, -1, 1), scale);
        WaypointSE2 p3 = new WaypointSE2(
                new Pose2d(new Translation2d(0, -1), Rotation2d.kCCW_90deg),
                new DirectionSE2(1, 0, 1), scale);
        HolonomicSpline s0 = new HolonomicSpline(p0, p1);
        HolonomicSpline s1 = new HolonomicSpline(p1, p2);
        HolonomicSpline s2 = new HolonomicSpline(p2, p3);
        HolonomicSpline s3 = new HolonomicSpline(p3, p0);
        List<HolonomicSpline> splines = List.of(s0, s1, s2, s3);
        checkCircle(splines, 0.008, 0.006);
        TrajectoryPlotter.plot(splines, 0.1);

    }

    @Test
    void testDheading() {
        // does the spline-derived dheading match the post-hoc one?
        double scale = 0.9;
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        HolonomicSpline spline = new HolonomicSpline(w0, w1);
        Pose2dWithMotion p0 = spline.getPose2dWithMotion(0.0);
        if (DEBUG)
            System.out.println(
                    "s, p0_heading_rate, p0_curvature, distance, post_hoc_heading_rate, post_hoc_curvature, post_hoc_heading_rate2, post_hoc_curvature2");
        for (double s = 0.01; s <= 1.0; s += 0.01) {
            Pose2dWithMotion p1 = spline.getPose2dWithMotion(s);
            double cartesianDistance = p1.distanceCartesian(p0);
            Rotation2d heading0 = p0.getPose().pose().getRotation();
            Rotation2d heading1 = p1.getPose().pose().getRotation();
            double dheading = heading1.minus(heading0).getRadians();
            DirectionSE2 course0 = p0.getPose().course();
            DirectionSE2 course1 = p1.getPose().course();
            double curve = Metrics.translationalNorm(course1.minus(course0));
            // this value matches the intrinsic one since it just uses
            // cartesian distance in the denominator.
            double dheadingDx2 = dheading / cartesianDistance;
            double curveDx2 = curve / cartesianDistance;

            if (DEBUG)
                System.out.printf(
                        "%5.3f, %5.3f, %5.3f, %5.3f, %5.3f \n",
                        s, p0.getHeadingRateRad_M(), p0.getCurvatureRad_M(),
                        dheadingDx2, curveDx2);
            p0 = p1;
        }

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
        assertEquals(0, actualRangeError, rangeError,
                String.format("range actual %f allowed %f", actualRangeError, rangeError));
        assertEquals(0, actualAzimuthError, azimuthError,
                String.format("azimuth actual %f expected %f", actualAzimuthError, azimuthError));

    }

    @Test
    void testLine() {
        // turn a bit to the left
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 0), 1));
        // turn much more to the left
        HolonomicSpline s1 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(2, 0),
                                Rotation2d.k180deg),
                        new DirectionSE2(1, 0, 0), 1));
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(s0);
        splines.add(s1);
        TrajectoryPlotter.plot(splines, 0.1);
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
        TrajectoryPlotter.plot(splines, 0.1);
    }

    @Test
    void testMismatchedXYDerivatives() {
        // because path generation never looks across spline boundaries,
        // it is possible to make sharp corners at the "knots."  But you can't
        // make a trajectory with these corners, the scheduler will fail.

        // this goes straight ahead to (1,0)
        // derivatives point straight ahead
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1));
        // this is a sharp turn to the left
        // derivatives point to the left
        HolonomicSpline s1 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 1, 0), 1));
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(s0);
        splines.add(s1);

        TrajectoryPlotter.plot(splines, 0.1);

        for (HolonomicSpline s : splines) {
            if (DEBUG)
                System.out.printf("spline %s\n", s);
        }

        Path100 path = new Path100(PathFactory.parameterizeSplines(splines, 0.05, 0.05, 0.05));
        if (DEBUG)
            System.out.printf("path %s\n", path);
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(logger, 1, 1));
        ScheduleGenerator scheduleGenerator = new ScheduleGenerator(constraints);
        assertThrows(IllegalStateException.class, () -> scheduleGenerator.timeParameterizeTrajectory(path,
                0.05, 0, 0));
    }

    @Test
    void testEntryVelocity() {

        // radius is 1 m.
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, -1),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 1, 0), 1.2));
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
                System.out.printf("%5.3f %5.3f\n", p.getPose().pose().getTranslation().getX(),
                        p.getPose().pose().getTranslation().getY());
            }
        }
        Path100 path = new Path100(motion);
        if (DEBUG) {
            for (int i = 0; i < path.length(); ++i) {
                System.out.printf("%5.3f %5.3f\n",
                        path.getPoint(i).getPose().pose().getTranslation().getX(),
                        path.getPoint(i).getPose().pose().getTranslation().getY());
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
}
