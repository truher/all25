package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactory {
    private static final boolean DEBUG = false;

    public static Path100 pathFromWaypoints(
            List<WaypointSE2> waypoints,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new HolonomicSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        return new Path100(parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    /**
     * Converts a spline into a list of Pose2dWithMotion.
     * 
     * The points are chosen so that the secant line between the points is within
     * the specified tolerance (dx, dy, dtheta) of the actual spline.
     * 
     * The trajectory scheduler consumes these points, interpolating between them
     * with straight lines. It might be better to sample the spline directly.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2dWithCurvature that approximates the original spline
     */
    static List<Pose2dWithMotion> parameterizeSpline(
            HolonomicSpline s,
            double maxDx,
            double maxDy,
            double maxDTheta,
            double t0,
            double t1) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        rv.add(s.getPose2dWithMotion(0.0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt) {
            PathFactory.getSegmentArc(s, rv, t, t + dt, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }

    public static List<Pose2dWithMotion> parameterizeSplines(
            List<? extends HolonomicSpline> splines,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        if (splines.isEmpty())
            return rv;
        rv.add(splines.get(0).getPose2dWithMotion(0.0));
        for (int i = 0; i < splines.size(); i++) {
            HolonomicSpline s = splines.get(i);
            if (DEBUG)
                System.out.printf("SPLINE:\n%d\n%s\n", i, s);
            List<Pose2dWithMotion> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    /**
     * Recursive bisection to find a series of secant lines close to the real curve.
     * 
     * Note if the path is s-shaped, then bisection can find the middle :-)
     */
    private static void getSegmentArc(
            HolonomicSpline spline,
            List<Pose2dWithMotion> rv,
            double t0, // [0,1] not time
            double t1, // [0,1] not time
            double maxDx,
            double maxDy,
            double maxDTheta) {
        // points must be this close together
        // TODO: make this a parameter.
        final double maxNorm = 0.1;
        Pose2d p0 = spline.getPose2d(t0);
        double thalf = (t0 + t1) / 2;
        Pose2d phalf = spline.getPose2d(thalf);
        Pose2d p1 = spline.getPose2d(t1);

        // twist from p0 to p1
        Twist2d twist_full = p0.log(p1);
        // twist halfway from p0 to p1
        Twist2d twist_half = GeometryUtil.scale(twist_full, 0.5);
        // point halfway from p0 to p1
        Pose2d phalf_predicted = p0.exp(twist_half);
        // difference between twist and sample
        Transform2d error = phalf_predicted.minus(phalf);

        Pose2dWithMotion p20 = spline.getPose2dWithMotion(t0);
        Pose2dWithMotion p21 = spline.getPose2dWithMotion(t1);
        Twist2d p2t = p20.getPose().course().minus(p21.getPose().course());

        // checks both translational and l2 norms
        // also checks change in course
        if (Math.abs(error.getTranslation().getX()) > maxDx
                || Math.abs(error.getTranslation().getY()) > maxDy
                || Math.abs(error.getRotation().getRadians()) > maxDTheta
                || Metrics.translationalNorm(twist_full) > maxNorm
                || Metrics.l2Norm(twist_full) > maxNorm
                || Metrics.l2Norm(p2t) > maxNorm) {
            // add a point in between
            // note the extra condition to avoid points too far apart.
            getSegmentArc(spline, rv, t0, thalf, maxDx, maxDy, maxDTheta);
            getSegmentArc(spline, rv, thalf, t1, maxDx, maxDy, maxDTheta);
        } else {
            // midpoint is close enough, this looks good
            rv.add(spline.getPose2dWithMotion(t1));
        }
    }
}
