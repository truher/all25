package org.team100.lib.trajectory.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;
import org.team100.lib.trajectory.timing.ScheduleGenerator.TimingException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class Path100Test {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;

    private static final List<Rotation2d> HEADINGS = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90));

    private static final List<Pose2dWithMotion> WAYPOINTS = Arrays.asList(
            new Pose2dWithMotion(
                    WaypointSE2.irrotational(
                            new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                    0, 0),
            new Pose2dWithMotion(
                    WaypointSE2.irrotational(
                            new Pose2d(24, 0, new Rotation2d(Math.toRadians(30))), 0, 1.2),
                    0, 0),
            new Pose2dWithMotion(
                    WaypointSE2.irrotational(
                            new Pose2d(36, 12, new Rotation2d(Math.toRadians(60))), 0, 1.2),
                    0, 0),
            new Pose2dWithMotion(
                    WaypointSE2.irrotational(
                            new Pose2d(60, 12, new Rotation2d(Math.toRadians(90))), 0, 1.2),
                    0, 0));

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

        assertEquals(HEADINGS.get(0), traj.getPoint(0).getPose().pose().getRotation());
        assertEquals(HEADINGS.get(1), traj.getPoint(1).getPose().pose().getRotation());
        assertEquals(HEADINGS.get(2), traj.getPoint(2).getPose().pose().getRotation());
        assertEquals(HEADINGS.get(3), traj.getPoint(3).getPose().pose().getRotation());
    }

    /**
     * Note that many of the results here are "wrong" because the waypoints aren't
     * correctly specified.
     */
    @Test
    void test() throws TimingException {
        List<Pose2dWithMotion> waypoints = Arrays.asList(
                new Pose2dWithMotion(
                        WaypointSE2.irrotational(
                                new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0))), 0, 1.2),
                        0.1, 0),
                new Pose2dWithMotion(
                        WaypointSE2.irrotational(
                                new Pose2d(24.0, 0.0, new Rotation2d(Math.toRadians(30))), 0, 1.2),
                        0.1, 0),
                new Pose2dWithMotion(
                        WaypointSE2.irrotational(
                                new Pose2d(36.0, 0.0, new Rotation2d(Math.toRadians(60))), Math.PI / 2, 1.2),
                        1e6, 0),
                new Pose2dWithMotion(
                        WaypointSE2.irrotational(
                                new Pose2d(36.0, 24.0, new Rotation2d(Math.toRadians(60))), 0, 1.2),
                        0.1, 0),
                new Pose2dWithMotion(
                        WaypointSE2.irrotational(
                                new Pose2d(60.0, 24.0, new Rotation2d(Math.toRadians(180))), 0, 1.2),
                        0.1, 0));

        // Create the reference trajectory (straight line motion between waypoints).
        Path100 path = new Path100(waypoints);

        if (DEBUG) {
            System.out.println("d, x, y, heading, course");
            for (int d = 0; d < 90; ++d) {
                Pose2dWithMotion s = path.sample(d);
                Pose2d p = s.getPose().pose();
                System.out.printf("%d, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        d, p.getX(), p.getY(), p.getRotation().getRadians(),
                        s.getPose().course().toRotation().getRadians());
            }
        }

        // constant-twist arcs
        assertEquals(84.108, path.getMaxDistance(), DELTA);

        // initial sample is exactly at the start
        Pose2dWithMotion sample0 = path.sample(0.0);
        assertEquals(0, sample0.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, sample0.getPose().pose().getTranslation().getY(), DELTA);

        // course is +x
        assertEquals(0, sample0.getPose().course().toRotation().getDegrees());

        // heading is 0
        assertEquals(0, sample0.getPose().pose().getRotation().getDegrees());

        Pose2dWithMotion sample12 = path.sample(12.0);
        assertEquals(11.997, sample12.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(0, sample12.getPose().pose().getTranslation().getY(), DELTA);

        // course should be +x
        assertEquals(0, sample12.getPose().course().toRotation().getDegrees(), DELTA);

        assertEquals(14.996, sample12.getPose().pose().getRotation().getDegrees(), DELTA);

        Pose2dWithMotion sample5 = path.sample(48);
        assertEquals(36, sample5.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(11.983, sample5.getPose().pose().getTranslation().getY(), DELTA);
        assertEquals(45.082, sample5.getPose().course().toRotation().getDegrees(), DELTA);
        assertEquals(60, sample5.getPose().pose().getRotation().getDegrees(), DELTA);

        Pose2dWithMotion sample6 = path.sample(60);
        assertEquals(36, sample6.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(23.983, sample6.getPose().pose().getTranslation().getY(), DELTA);
        assertEquals(0.041, sample6.getPose().course().toRotation().getDegrees(), DELTA);
        assertEquals(60, sample6.getPose().pose().getRotation().getDegrees(), DELTA);

        Pose2dWithMotion sample72 = path.sample(72.0);
        assertEquals(47.937, sample72.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(24, sample72.getPose().pose().getTranslation().getY(), DELTA);

        // course should be +x
        assertEquals(0, sample72.getPose().course().toRotation().getDegrees(), DELTA);

        assertEquals(119.687, sample72.getPose().pose().getRotation().getDegrees(), DELTA);

        Pose2dWithMotion sample8 = path.sample(84);
        assertEquals(59.892, sample8.getPose().pose().getTranslation().getX(), DELTA);
        assertEquals(24, sample8.getPose().pose().getTranslation().getY(), DELTA);
        assertEquals(0, sample8.getPose().course().toRotation().getDegrees(), DELTA);
        assertEquals(179.460, sample8.getPose().pose().getRotation().getDegrees(), DELTA);

    }
}
