package org.team100.lib.trajectory.path.spline;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.Pose2dWithDirection;
import org.team100.lib.trajectory.TrajectoryPlotter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class QuinticHermiteOptimizerTest {

    @Test
    void test0() {
        Pose2dWithDirection a = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 100),
                        new Rotation2d()),
                DirectionSE2.MINUS_Y);
        Pose2dWithDirection b = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d()),
                DirectionSE2.TO_X);
        Pose2dWithDirection c = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 100),
                        new Rotation2d()),
                DirectionSE2.TO_Y);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b));
        splines.add(new HolonomicSpline(b, c));

        TrajectoryPlotter.plot(splines, 5, 10);

    }

    @Test
    void test1() {

        Pose2dWithDirection d = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                DirectionSE2.TO_Y);
        Pose2dWithDirection e = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 50),
                        new Rotation2d()),
                DirectionSE2.TO_X);
        Pose2dWithDirection f = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d()),
                DirectionSE2.MINUS_Y);
        Pose2dWithDirection g = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 0),
                        new Rotation2d()),
                DirectionSE2.MINUS_X);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(d, e));
        splines.add(new HolonomicSpline(e, f));
        splines.add(new HolonomicSpline(f, g));

        TrajectoryPlotter.plot(splines, 5, 10);

    }

    @Test
    void test2() {
        Pose2dWithDirection h = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                DirectionSE2.TO_X);
        Pose2dWithDirection i = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d()),
                DirectionSE2.TO_X);
        Pose2dWithDirection j = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d()),
                new DirectionSE2(1, 1, 0));
        Pose2dWithDirection k = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(150, 0),
                        new Rotation2d()),
                DirectionSE2.MINUS_Y);
        Pose2dWithDirection l = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(150, -50),
                        new Rotation2d()),
                DirectionSE2.MINUS_Y);

        List<HolonomicSpline> splines2 = new ArrayList<>();
        splines2.add(new HolonomicSpline(h, i));
        splines2.add(new HolonomicSpline(i, j));
        splines2.add(new HolonomicSpline(j, k));
        splines2.add(new HolonomicSpline(k, l));

        TrajectoryPlotter.plot(splines2, 5, 10);

    }

    @Test
    void test3() {
        Pose2dWithDirection a = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 100),
                        new Rotation2d()),
                DirectionSE2.MINUS_Y);
        Pose2dWithDirection b = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d(Math.PI / 2)),
                DirectionSE2.TO_X);
        Pose2dWithDirection c = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 100),
                        new Rotation2d(Math.PI)),
                DirectionSE2.TO_Y);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(a, b));
        splines.add(new HolonomicSpline(b, c));

        TrajectoryPlotter.plot(splines, 5, 10);

    }

    @Test
    void test4() {

        Pose2dWithDirection d = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                DirectionSE2.TO_Y);
        Pose2dWithDirection e = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 50),
                        new Rotation2d(Math.PI / 2)),
                DirectionSE2.TO_X);
        Pose2dWithDirection f = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d(Math.PI)),
                DirectionSE2.MINUS_Y);
        Pose2dWithDirection g = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 0),
                        new Rotation2d()),
                DirectionSE2.MINUS_X);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(d, e));
        splines.add(new HolonomicSpline(e, f));
        splines.add(new HolonomicSpline(f, g));

        TrajectoryPlotter.plot(splines, 5, 10);
    }

    @Test
    void test5() {

        Pose2dWithDirection h = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(0, 0),
                        new Rotation2d()),
                DirectionSE2.TO_X);
        Pose2dWithDirection i = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(50, 0),
                        new Rotation2d(Math.PI / 2)),
                DirectionSE2.TO_X);
        Pose2dWithDirection j = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(100, 50),
                        new Rotation2d(Math.PI)),
                new DirectionSE2(1, 1, 0));
        Pose2dWithDirection k = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(150, 0),
                        new Rotation2d()),
                DirectionSE2.MINUS_Y);
        Pose2dWithDirection l = new Pose2dWithDirection(
                new Pose2d(
                        new Translation2d(150, -50),
                        new Rotation2d(Math.PI / 2)),
                DirectionSE2.MINUS_Y);

        List<HolonomicSpline> splines = new ArrayList<>();
        splines.add(new HolonomicSpline(h, i));
        splines.add(new HolonomicSpline(i, j));
        splines.add(new HolonomicSpline(j, k));
        splines.add(new HolonomicSpline(k, l));

        TrajectoryPlotter.plot(splines, 5, 10);

    }
}
