package org.team100.lib.trajectory.path.spline;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.TrajectoryPlotter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class QuinticHermiteOptimizerTest {

    @Test
    void test0() {
        WaypointSE2 a = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 100),
                        new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 b = new WaypointSE2(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 c = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 100),
                        new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b));
        splines.add(new HolonomicSpline(b, c));

        TrajectoryPlotter.plot(splines, 5);

    }

    @Test
    void test1() {

        WaypointSE2 d = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        WaypointSE2 e = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 50),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 f = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 g = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 0),
                        new Rotation2d()),
                new DirectionSE2(-1, 0, 0), 1);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(d, e));
        splines.add(new HolonomicSpline(e, f));
        splines.add(new HolonomicSpline(f, g));

        TrajectoryPlotter.plot(splines, 5);

    }

    @Test
    void test2() {
        WaypointSE2 h = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 i = new WaypointSE2(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 j = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d()),
                new DirectionSE2(1, 1, 0), 1);
        WaypointSE2 k = new WaypointSE2(
                new Pose2d(
                        new Translation2d(150, 0),
                        new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 l = new WaypointSE2(
                new Pose2d(
                        new Translation2d(150, -50),
                        new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);

        List<HolonomicSpline> splines2 = new ArrayList<>();
        splines2.add(new HolonomicSpline(h, i));
        splines2.add(new HolonomicSpline(i, j));
        splines2.add(new HolonomicSpline(j, k));
        splines2.add(new HolonomicSpline(k, l));

        TrajectoryPlotter.plot(splines2, 5);

    }

    @Test
    void test3() {
        WaypointSE2 a = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 100),
                        new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 b = new WaypointSE2(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 c = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 100),
                        new Rotation2d(Math.PI)),
                new DirectionSE2(0, 1, 0), 1);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b));
        splines.add(new HolonomicSpline(b, c));

        TrajectoryPlotter.plot(splines, 5);

    }

    @Test
    void test4() {

        WaypointSE2 d = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        WaypointSE2 e = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 50),
                        new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 f = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d(Math.PI)),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 g = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 0),
                        new Rotation2d()),
                new DirectionSE2(-1, 0, 0), 1);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(d, e));
        splines.add(new HolonomicSpline(e, f));
        splines.add(new HolonomicSpline(f, g));

        TrajectoryPlotter.plot(splines, 5);
    }

    @Test
    void test5() {

        WaypointSE2 h = new WaypointSE2(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 i = new WaypointSE2(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 j = new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d(Math.PI)),
                new DirectionSE2(1, 1, 0), 1);
        WaypointSE2 k = new WaypointSE2(
                new Pose2d(
                        new Translation2d(150, 0),
                        new Rotation2d()),
                new DirectionSE2(0, -1, 0), 1);
        WaypointSE2 l = new WaypointSE2(
                new Pose2d(
                        new Translation2d(150, -50),
                        new Rotation2d(Math.PI / 2)),
                new DirectionSE2(0, -1, 0), 1);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(h, i));
        splines.add(new HolonomicSpline(i, j));
        splines.add(new HolonomicSpline(j, k));
        splines.add(new HolonomicSpline(k, l));

        TrajectoryPlotter.plot(splines, 5);

    }
}
