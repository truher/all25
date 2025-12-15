package org.team100.lib.trajectory.path.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.Pose2dWithDirection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class QuinticHermiteOptimizerTest {
    private static double kEpsilon = 1e-12;

    @Test
    void test() {
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

        assertTrue(SplineUtil.optimizeSpline(splines) < 0.014);

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

        List<HolonomicSpline> splines1 = new ArrayList<>();
        splines1.add(new HolonomicSpline(d, e));
        splines1.add(new HolonomicSpline(e, f));
        splines1.add(new HolonomicSpline(f, g));

        assertEquals(0.54, SplineUtil.optimizeSpline(splines1), 0.01);

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

        assertTrue(SplineUtil.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);
    }

    @Test
    void testHolonomic() {
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

        assertTrue(SplineUtil.optimizeSpline(splines) < 0.014);

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

        List<HolonomicSpline> splines1 = new ArrayList<>();
        splines1.add(new HolonomicSpline(d, e));
        splines1.add(new HolonomicSpline(e, f));
        splines1.add(new HolonomicSpline(f, g));

        assertEquals(0.54, SplineUtil.optimizeSpline(splines1), 0.01);

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

        List<HolonomicSpline> splines2 = new ArrayList<>();
        splines2.add(new HolonomicSpline(h, i));
        splines2.add(new HolonomicSpline(i, j));
        splines2.add(new HolonomicSpline(j, k));
        splines2.add(new HolonomicSpline(k, l));

        assertTrue(SplineUtil.optimizeSpline(splines2) < 0.05);
        assertEquals(0.0, splines2.get(0).getCurvature(1.0), kEpsilon);
        assertEquals(0.0, splines2.get(2).getCurvature(1.0), kEpsilon);

    }

}
