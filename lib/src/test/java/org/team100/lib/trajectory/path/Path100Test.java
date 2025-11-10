package org.team100.lib.trajectory.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;
import org.team100.lib.trajectory.timing.ScheduleGenerator.TimingException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class Path100Test {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = true;

    private static final List<Rotation2d> HEADINGS = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90));

    private static final List<Pose2dWithMotion> WAYPOINTS = Arrays.asList(
            new Pose2dWithMotion(
                    HolonomicPose2d.make(0, 0, 0, 0), 0, 0, 0),
            new Pose2dWithMotion(
                    HolonomicPose2d.make(24, 0, Math.toRadians(30), 0), 0, 0, 0),
            new Pose2dWithMotion(
                    HolonomicPose2d.make(36, 12, Math.toRadians(60), 0), 0, 0, 0),
            new Pose2dWithMotion(
                    HolonomicPose2d.make(60, 12, Math.toRadians(90), 0), 0, 0, 0));

    @Test
    void testEmpty() {
        List<HolonomicSpline> splines = new ArrayList<>();
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(0, path.length(), 0.001);
    }

    @Test
    void testSimple() {
        // spline is in the x direction, no curvature.
        HolonomicSpline spline = new HolonomicSpline(
                new HolonomicPose2d(
                        new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(
                        new Translation2d(1, 0), new Rotation2d(), new Rotation2d())) {

            @Override
            public Translation2d getPoint(double t) {
                return new Translation2d(t, 0);
            }

            @Override
            public Rotation2d getHeading(double t) {
                return Rotation2d.kZero;
            }

            @Override
            public Optional<Rotation2d> getCourse(double t) {
                return Optional.of(Rotation2d.kZero);
            }

            @Override
            public double getDHeading(double t) {
                return 0;
            }

            @Override
            public double getCurvature(double t) {
                return 0;
            }

            @Override
            public double getDCurvature(double t) {
                return 0;
            }

            @Override
            public double getVelocity(double t) {
                return 1;
            }
        };
        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(spline);
        double maxDx = 0.1;
        double maxDy = 0.1;
        double maxDTheta = 0.1;
        Path100 path = new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
        assertEquals(2, path.length(), 0.001);
    }

    @Test
    void testConstruction() {
        Path100 traj = new Path100(WAYPOINTS);
        assertFalse(traj.isEmpty());
        assertEquals(4, traj.length());
    }

    @Test
    void testStateAccessors() {
        Path100 traj = new Path100(WAYPOINTS);

        assertEquals(WAYPOINTS.get(0), traj.getPoint(0));
        assertEquals(WAYPOINTS.get(1), traj.getPoint(1));
        assertEquals(WAYPOINTS.get(2), traj.getPoint(2));
        assertEquals(WAYPOINTS.get(3), traj.getPoint(3));

        assertEquals(HEADINGS.get(0), traj.getPoint(0).getPose().heading());
        assertEquals(HEADINGS.get(1), traj.getPoint(1).getPose().heading());
        assertEquals(HEADINGS.get(2), traj.getPoint(2).getPose().heading());
        assertEquals(HEADINGS.get(3), traj.getPoint(3).getPose().heading());
    }

    /**
     * Note that many of the results here are "wrong" because the waypoints aren't
     * correctly specified.
     */
    @Test
    void test() throws TimingException {
        List<Pose2dWithMotion> waypoints = Arrays.asList(
                new Pose2dWithMotion(
                        HolonomicPose2d.make(0.0, 0.0, Math.toRadians(0), 0),
                        0.1, 0, 0),
                new Pose2dWithMotion(
                        HolonomicPose2d.make(24.0, 0.0, Math.toRadians(30), 0),
                        0.1, 0, 0),
                new Pose2dWithMotion(
                        HolonomicPose2d.make(36.0, 0.0, Math.toRadians(60), Math.PI / 2),
                        1e6, 0, 0),
                new Pose2dWithMotion(
                        HolonomicPose2d.make(36.0, 24.0, Math.toRadians(60), 0),
                        0.1, 0, 0),
                new Pose2dWithMotion(
                        HolonomicPose2d.make(60.0, 24.0, Math.toRadians(180), 0),
                        0.1, 0, 0));

        // Create the reference trajectory (straight line motion between waypoints).
        Path100 path = new Path100(waypoints);

        if (DEBUG) {
            System.out.println("d, x, y, heading, course");
            for (int d = 0; d < 90; ++d) {
                Pose2dWithMotion s = path.sample(d);
                Pose2d p = s.getPose().pose();
                System.out.printf("%d, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        d, p.getX(), p.getY(), p.getRotation().getRadians(), s.getCourse().getRadians());
            }
        }

        assertEquals(0.0, path.getMinDistance(), DELTA);
        // paths no longer use constant-twist arcs so this is just the lengths
        assertEquals(84, path.getMaxDistance(), DELTA);

        // initial sample is exactly at the start
        Pose2dWithMotion sample0 = path.sample(0.0);
        assertEquals(0, sample0.getPose().translation().getX(), DELTA);
        assertEquals(0, sample0.getPose().translation().getY(), DELTA);

        // course is +x
        assertEquals(0, sample0.getCourse().getDegrees());

        // heading is 0
        assertEquals(0, sample0.getPose().heading().getDegrees());

        Pose2dWithMotion sample12 = path.sample(12.0);
        assertEquals(12, sample12.getPose().translation().getX(), DELTA);
        assertEquals(0, sample12.getPose().translation().getY(), DELTA);

        // course should be +x
        assertEquals(0, sample12.getCourse().getDegrees(), DELTA);

        assertEquals(15, sample12.getPose().heading().getDegrees(), DELTA);

        Pose2dWithMotion sample5 = path.sample(48);
        assertEquals(36, sample5.getPose().translation().getX(), DELTA);
        assertEquals(12, sample5.getPose().translation().getY(), DELTA);
        assertEquals(45, sample5.getCourse().getDegrees(), DELTA);
        assertEquals(60, sample5.getPose().heading().getDegrees(), DELTA);

        Pose2dWithMotion sample6 = path.sample(60);
        assertEquals(36, sample6.getPose().translation().getX(), DELTA);
        assertEquals(24, sample6.getPose().translation().getY(), DELTA);
        assertEquals(0, sample6.getCourse().getDegrees(), DELTA);
        assertEquals(60, sample6.getPose().heading().getDegrees(), DELTA);

        Pose2dWithMotion sample72 = path.sample(72.0);
        assertEquals(48, sample72.getPose().translation().getX(), DELTA);
        assertEquals(24, sample72.getPose().translation().getY(), DELTA);

        // course should be +x
        assertEquals(0, sample72.getCourse().getDegrees(), DELTA);

        assertEquals(120, sample72.getPose().heading().getDegrees(), DELTA);

        Pose2dWithMotion sample8 = path.sample(84);
        assertEquals(60, sample8.getPose().translation().getX(), DELTA);
        assertEquals(24, sample8.getPose().translation().getY(), DELTA);
        assertEquals(0, sample8.getCourse().getDegrees(), DELTA);
        assertEquals(180, sample8.getPose().heading().getDegrees(), DELTA);

    }
}
