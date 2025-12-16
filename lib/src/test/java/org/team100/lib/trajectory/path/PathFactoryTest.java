package org.team100.lib.trajectory.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.TrajectoryPlotter;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactoryTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.01;

    @Test
    void testBackingUp() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1));
        Path100 path = PathFactory.pathFromWaypoints(
                waypoints,
                0.0127,
                0.0127,
                Math.toRadians(1.0));
        assertEquals(10, path.length());
    }

    /** Preserves the tangent at the corner and so makes a little "S" */
    @Test
    void testCorner() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1));
        Path100 path = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);

        assertEquals(9, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
        p = path.getPoint(1);
        assertEquals(1, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
    }

    @Test
    void testLinear() {
        List<WaypointSE2> waypoints = List.of(
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
        Path100 path = PathFactory.pathFromWaypoints(
                waypoints, 0.01, 0.01, 0.1);
        assertEquals(2, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
        p = path.getPoint(1);
        assertEquals(1, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getPose().heading().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
    }

    /** Turning in place works now. */
    @Test
    void testSpin() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 0, 1), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kCCW_90deg),
                        new DirectionSE2(0, 0, 1), 1));
        Path100 path = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);
        TrajectoryPlotter.plot(path, 0.1, 1);
    }

    /** Hard corners work now. */
    @Test
    void testActualCorner() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1));
        Path100 path = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);
        TrajectoryPlotter.plot(path, 0.1, 1);
    }

    @Test
    void testComposite() {
        // note none of these directions include rotation; it's all in between.
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(2, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 0), 1));
        Path100 foo = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);
        TrajectoryPlotter.plot(foo, 0.1, 1);
        assertEquals(4, foo.length(), 0.001);
    }

    @Test
    void test() {
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 0),
                        Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1.2);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(
                        new Translation2d(15, 10),
                        Rotation2d.kZero),
                new DirectionSE2(1, 5, 0), 1.2);
        HolonomicSpline s = new HolonomicSpline(p1, p2);

        List<Pose2dWithMotion> samples = PathFactory.parameterizeSpline(s, 0.05, 0.05, 0.1, 0.0, 1.0);

        double arclength = 0;
        Pose2dWithMotion cur_pose = samples.get(0);
        for (Pose2dWithMotion sample : samples) {
            Twist2d twist = GeometryUtil.slog(
                    GeometryUtil.transformBy(
                            GeometryUtil.inverse(
                                    cur_pose.getPose().pose()),
                            sample.getPose().pose()));
            arclength += Math.hypot(twist.dx, twist.dy);
            cur_pose = sample;
        }

        assertEquals(15.0, cur_pose.getPose().translation().getX(), 0.001);
        assertEquals(10.0, cur_pose.getPose().translation().getY(), 0.001);
        assertEquals(78.690, cur_pose.getCourse().getDegrees(), 0.001);
        assertEquals(20.416, arclength, 0.001);
    }

    @Test
    void testDx() {
        HolonomicSpline s0 = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, -1),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 1, 0), 1));
        List<HolonomicSpline> splines = List.of(s0);
        List<Pose2dWithMotion> motion = PathFactory.parameterizeSplines(splines, 0.001, 0.001, 0.001);
        for (Pose2dWithMotion p : motion) {
            if (DEBUG)
                System.out.printf("%5.3f %5.3f\n", p.getPose().translation().getX(), p.getPose().translation().getY());
        }
    }

    /**
     * 0.15 ms on my machine.
     * 
     * See TrajectoryPlannerTest::testPerformance().
     */
    @Test
    void testPerformance() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1.2));
        long startTimeNs = System.nanoTime();
        Path100 t = new Path100(new ArrayList<>());
        final int iterations = 100;
        final double SPLINE_SAMPLE_TOLERANCE_M = 0.05;
        final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
        for (int i = 0; i < iterations; ++i) {
            t = PathFactory.pathFromWaypoints(
                    waypoints,
                    SPLINE_SAMPLE_TOLERANCE_M,
                    SPLINE_SAMPLE_TOLERANCE_M,
                    SPLINE_SAMPLE_TOLERANCE_RAD);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
            System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        }
        assertEquals(5, t.length());
        Pose2dWithMotion p = t.getPoint(1);
        assertEquals(0.417, p.getPose().translation().getX(), DELTA);
        assertEquals(0, p.getHeadingRateRad_M(), DELTA);
    }

}
