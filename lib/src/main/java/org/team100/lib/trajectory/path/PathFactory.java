package org.team100.lib.trajectory.path;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithDirection;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactory {
    private static final boolean DEBUG = false;

    public static Path100 pathFromWaypoints(
            List<Pose2dWithDirection> waypoints,
            double maxDx,
            double maxDy,
            double maxDTheta,
            List<Double> magicNumbers) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(
                    new HolonomicSpline(
                            waypoints.get(i - 1),
                            waypoints.get(i),
                            magicNumbers.get(i - 1),
                            magicNumbers.get(i)));
        }
        return new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    public static Path100 pathFromWaypoints(
            List<Pose2dWithDirection> waypoints,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            HolonomicSpline s = new HolonomicSpline(
                    waypoints.get(i - 1),
                    waypoints.get(i));
            if (DEBUG)
                System.out.printf("%d %s\n", i, s);
            splines.add(s);
        }
        return new Path100(PathFactory.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
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
                System.out.printf("%d %s\n", i, s);
            List<Pose2dWithMotion> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    /**
     * Recursively add points until they're close to the real curve.
     */
    private static void getSegmentArc(
            HolonomicSpline s,
            List<Pose2dWithMotion> rv,
            double t0,
            double t1,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        Pose2d p0 = s.getPose2d(t0);
        double thalf = (t0 + t1) / 2;
        Pose2d phalf = s.getPose2d(thalf);
        Pose2d p1 = s.getPose2d(t1);

        // twist from p0 to p1
        Twist2d twist_full = p0.log(p1);
        // twist halfway from p0 to p1
        Twist2d twist_half = GeometryUtil.scale(twist_full, 0.5);
        // point halfway from p0 to p1
        Pose2d phalf_predicted = p0.exp(twist_half);
        // difference between twist and sample
        Transform2d error = phalf_predicted.minus(phalf);

        if (Math.abs(error.getTranslation().getX()) > maxDx ||
                Math.abs(error.getTranslation().getY()) > maxDy ||
                Math.abs(error.getRotation().getRadians()) > maxDTheta) {
            // add a point in between
            getSegmentArc(s, rv, t0, thalf, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, thalf, t1, maxDx, maxDy, maxDTheta);
        } else {
            // midpoint is close enough, this looks good
            rv.add(s.getPose2dWithMotion(t1));
        }
    }
}
