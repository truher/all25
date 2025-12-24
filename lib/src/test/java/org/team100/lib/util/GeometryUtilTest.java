package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class GeometryUtilTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testCurvature() {
        HolonomicSpline spline = new HolonomicSpline(
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
        // verify one point
        {
            double splineCurvature = spline.getCurvature(0.5);
            assertEquals(0.950, splineCurvature, DELTA);
            Pose2d p0 = spline.getPose2d(0.49);
            Pose2d p1 = spline.getPose2d(0.5);
            Pose2d p2 = spline.getPose2d(0.51);
            double mengerCurvature = GeometryUtil.mengerCurvature(
                    p0.getTranslation(), p1.getTranslation(), p2.getTranslation());
            assertEquals(0.950, mengerCurvature, DELTA);
        }
        // verify all the points
        double DS = 0.01;
        for (double s = DS; s <= 1 - DS; s += DS) {
            double splineCurvature = spline.getCurvature(s);
            Pose2d p0 = spline.getPose2d(s - DS);
            Pose2d p1 = spline.getPose2d(s);
            Pose2d p2 = spline.getPose2d(s + DS);
            double mengerCurvature = GeometryUtil.mengerCurvature(
                    p0.getTranslation(), p1.getTranslation(), p2.getTranslation());
            if (DEBUG)
                System.out.printf("%f %f %f %f\n", s, splineCurvature, mengerCurvature,
                        splineCurvature - mengerCurvature);
            // error scales with ds.
            assertEquals(splineCurvature, mengerCurvature, 0.001);
        }
    }

    @Test
    void testCurvature2() {
        // no curve
        HolonomicSpline spline = new HolonomicSpline(
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
        double splineCurvature = spline.getCurvature(0.5);
        assertEquals(0, splineCurvature, DELTA);
        Pose2d p0 = spline.getPose2d(0.49);
        Pose2d p1 = spline.getPose2d(0.5);
        Pose2d p2 = spline.getPose2d(0.51);
        double mengerCurvature = GeometryUtil.mengerCurvature(
                p0.getTranslation(), p1.getTranslation(), p2.getTranslation());
        assertEquals(0, mengerCurvature, DELTA);
    }

    @Test
    void testCurvature3() {
        // turn in place
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d()),
                new DirectionSE2(0, 0, 1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(1)),
                new DirectionSE2(0, 0, 1), 1);
        HolonomicSpline spline = new HolonomicSpline(w0, w1);

        double splineCurvature = spline.getCurvature(0.5);
        assertEquals(0, splineCurvature, DELTA);
        Pose2d p0 = spline.getPose2d(0.49);
        Pose2d p1 = spline.getPose2d(0.5);
        Pose2d p2 = spline.getPose2d(0.51);
        double mengerCurvature = GeometryUtil.mengerCurvature(
                p0.getTranslation(), p1.getTranslation(), p2.getTranslation());
        assertEquals(0, mengerCurvature, DELTA);
    }

    @Test
    void testHeadingRate() {
        // note spline rotation rate is not constant, to make it more interesting
        HolonomicSpline spline = new HolonomicSpline(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d(1)),
                        new DirectionSE2(1, 0, 1), 1));
        {
            double splineHR = spline.getDHeadingDs(0.5);
            assertEquals(0.869, splineHR, DELTA);
            Pose2d p0 = spline.getPose2d(0.49);
            Pose2d p1 = spline.getPose2d(0.51);
            double discreteHR = GeometryUtil.headingRatio(p0, p1);
            assertEquals(0.869, discreteHR, DELTA);
        }
        double DS = 0.001;
        for (double s = DS; s <= 1 - DS; s += DS) {
            double splineHR = spline.getDHeadingDs(s);
            Pose2d p0 = spline.getPose2d(s - DS);
            Pose2d p1 = spline.getPose2d(s + DS);
            double discreteHR = GeometryUtil.headingRatio(p0, p1);
            if (DEBUG)
                System.out.printf("%f %f %f %f\n", s, splineHR, discreteHR, splineHR - discreteHR);
            // error scales with ds
            assertEquals(splineHR, discreteHR, 0.00001);
        }
    }

    @Test
    void testHeadingRate2() {
        // turning in place
        HolonomicSpline spline = new HolonomicSpline(
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
        {
            double splineHR = spline.getDHeadingDs(0.5);
            // this is heading change per L2 metric; pure rotation = 1
            assertEquals(1, splineHR, DELTA);
            Pose2d p0 = spline.getPose2d(0.49);
            Pose2d p1 = spline.getPose2d(0.51);
            double discreteHR = GeometryUtil.headingRatio(p0, p1);
            assertEquals(1, discreteHR, DELTA);
        }
        double DS = 0.001;
        for (double s = DS; s <= 1 - DS; s += DS) {
            double splineHR = spline.getDHeadingDs(s);
            Pose2d p0 = spline.getPose2d(s - DS);
            Pose2d p1 = spline.getPose2d(s + DS);
            double discreteHR = GeometryUtil.headingRatio(p0, p1);
            if (DEBUG)
                System.out.printf("%f %f %f %f\n", s, splineHR, discreteHR, splineHR - discreteHR);
            // error scales with ds
            assertEquals(splineHR, discreteHR, 0.00001);
        }
    }

    @Test
    void testProject() {
        {
            ChassisSpeeds prev = new ChassisSpeeds(0.5, 0.5, 0);
            ChassisSpeeds target = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(0.5, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(0, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            ChassisSpeeds prev = new ChassisSpeeds(0.5, 0.5, 1);
            ChassisSpeeds target = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(0.5, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(1, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            ChassisSpeeds prev = new ChassisSpeeds(0.5, 0.5, 1);
            ChassisSpeeds target = new ChassisSpeeds(2, 0, 1);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(0.5, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(1, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            ChassisSpeeds prev = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds target = new ChassisSpeeds(1, 0, 0);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(1, result.vxMetersPerSecond, 1e-12);
            assertEquals(0, result.vyMetersPerSecond, 1e-12);
            assertEquals(0, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(0, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
        {
            // project onto itself, no-op.
            ChassisSpeeds prev = new ChassisSpeeds(1.99996732, -0.01143327, 3.50000000);
            ChassisSpeeds target = new ChassisSpeeds(1.99996732, -0.01143327, 3.50000000);
            assertEquals(-0.005716666136460627, GeometryUtil.getCourse(target).get().getRadians(), 1e-12);
            ChassisSpeeds result = GeometryUtil.project(prev, target);
            assertEquals(1.99996732, result.vxMetersPerSecond, 1e-12);
            assertEquals(-0.01143327, result.vyMetersPerSecond, 1e-12);
            assertEquals(3.5, result.omegaRadiansPerSecond, 1e-12);
            assertEquals(-0.005716666136460627, GeometryUtil.getCourse(result).get().getRadians(), 1e-12);
        }
    }

    @Test
    void testSlog() {
        Twist2d twist = GeometryUtil.slog(
                new Pose2d(1, 0, Rotation2d.kZero));
        assertEquals(1, twist.dx, DELTA);
        assertEquals(0, twist.dy, DELTA);
        assertEquals(0, twist.dtheta, DELTA);

        // this twist represents an arc that ends up at the endpoint.
        twist = GeometryUtil.slog(
                new Pose2d(1, 0, Rotation2d.kCCW_Pi_2));
        assertEquals(0.785, twist.dx, DELTA);
        assertEquals(-0.785, twist.dy, DELTA);
        assertEquals(1.571, twist.dtheta, DELTA);
    }

    @Test
    void testDistance2d() {
        assertEquals(1, GeometryUtil.distanceM(
                new Translation2d(1, 0), new Translation2d(0, 0)), DELTA);
    }

    @Test
    void testDistance3d() {
        assertEquals(1, GeometryUtil.distanceM(
                new Translation3d(1, 0, 0), new Translation3d(0, 0, 0)), DELTA);
    }

    @Test
    void testDistance() {
        // same pose => 0
        assertEquals(0,
                GeometryUtil.distanceM(
                        new Pose2d(1, 0, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kZero)),
                DELTA);
        // 1d distance
        assertEquals(1,
                GeometryUtil.distanceM(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kZero)),
                DELTA);
        // 2d distance
        assertEquals(1.414,
                GeometryUtil.distanceM(
                        new Pose2d(0, 1, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kZero)),
                DELTA);
        // rotation means a little arc, so the path length is a little longer.
        assertEquals(1.111,
                GeometryUtil.distanceM(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kCCW_Pi_2)),
                DELTA);
        // the arc in this case is the entire quarter circle
        assertEquals(1.571,
                GeometryUtil.distanceM(
                        new Pose2d(0, 1, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kCCW_Pi_2)),
                DELTA);
        // order doesn't matter
        assertEquals(1.571,
                GeometryUtil.distanceM(
                        new Pose2d(1, 0, Rotation2d.kCCW_Pi_2),
                        new Pose2d(0, 1, Rotation2d.kZero)),
                DELTA);
        // pure rotation yields zero distance, which isn't really what we want.
        assertEquals(0,
                GeometryUtil.distanceM(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(0, 0, Rotation2d.kCCW_90deg)),
                DELTA);
        // use double geodesic distance to fix that.
        assertEquals(1.571,
                GeometryUtil.doubleGeodesicDistance(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(0, 0, Rotation2d.kCCW_90deg)),
                DELTA);
    }

    @Test
    void testCollinear() {
        assertTrue(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d())));
        assertFalse(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d(Math.PI))));
        assertTrue(GeometryUtil.isColinear(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 2)),
                new Pose2d(0, 1, new Rotation2d(Math.PI / 2))));
        assertFalse(GeometryUtil.isColinear(
                new Pose2d(),
                new Pose2d(1, 1, new Rotation2d())));
    }

    @Test
    void testParallel() {
        assertTrue(GeometryUtil.isParallel(new Rotation2d(), new Rotation2d()));
        assertTrue(GeometryUtil.isParallel(new Rotation2d(), new Rotation2d(Math.PI)));
    }

    @Test
    void testZforwardToXforward1() {

        // camera coordinates are x-right, y-down, z-forward
        // zero rotation.
        Rotation3d zforward = new Rotation3d();
        Quaternion q = zforward.getQuaternion();
        assertEquals(0, q.getX(), DELTA);
        assertEquals(0, q.getY(), DELTA);
        assertEquals(0, q.getZ(), DELTA);
        assertEquals(1, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is still zero.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward2() {

        // camera coordinates are x-right, y-down, z-forward
        // 45 degree rotation around z.
        Rotation3d zforward = new Rotation3d(0, 0, Math.PI / 4);
        Quaternion q = zforward.getQuaternion();
        assertEquals(0, q.getX(), DELTA);
        assertEquals(0, q.getY(), DELTA);
        assertEquals(0.383, q.getZ(), DELTA);
        assertEquals(0.924, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is around x.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(Math.PI / 4, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward3() {

        // camera coordinates are x-right, y-down, z-forward
        // 45 degree rotation around x. (tilt up)
        Rotation3d zforward = new Rotation3d(Math.PI / 4, 0, 0);
        Quaternion q = zforward.getQuaternion();
        assertEquals(0.383, q.getX(), DELTA);
        assertEquals(0, q.getY(), DELTA);
        assertEquals(0, q.getZ(), DELTA);
        assertEquals(0.924, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is around y, negative.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(-Math.PI / 4, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward4() {
        // camera coordinates are x-right, y-down, z-forward
        // 45 degree rotation around y. (pan right)
        Rotation3d zforward = new Rotation3d(0, Math.PI / 4, 0);
        Quaternion q = zforward.getQuaternion();
        assertEquals(0, q.getX(), DELTA);
        assertEquals(0.383, q.getY(), DELTA);
        assertEquals(0, q.getZ(), DELTA);
        assertEquals(0.924, q.getW(), DELTA);

        // robot coordinates are x-forward, y-left, z-up
        // in this frame the rotation is around y, negative.
        Rotation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(-Math.PI / 4, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward5() {
        Translation3d zforward = new Translation3d();
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward6() {
        Translation3d zforward = new Translation3d(0, 0, 1);
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(1, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward7() {
        Translation3d zforward = new Translation3d(1, 0, 0);
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(-1, xforward.getY(), DELTA);
        assertEquals(0, xforward.getZ(), DELTA);
    }

    @Test
    void testZforwardToXforward8() {
        Translation3d zforward = new Translation3d(0, 1, 0);
        // still zero
        Translation3d xforward = GeometryUtil.zForwardToXForward(zforward);
        assertEquals(0, xforward.getX(), DELTA);
        assertEquals(0, xforward.getY(), DELTA);
        assertEquals(-1, xforward.getZ(), DELTA);
    }
}
