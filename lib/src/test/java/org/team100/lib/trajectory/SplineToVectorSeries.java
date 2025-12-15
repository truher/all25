package org.team100.lib.trajectory;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SplineToVectorSeries {

    /** Length of the vector indicating heading */
    private final double m_scale;

    public SplineToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public VectorSeries convert(String name, List<HolonomicSpline> splines) {
        VectorSeries s = new VectorSeries(name);
        for (HolonomicSpline spline : splines) {
            for (double j = 0; j < 0.99; j += 0.1) {
                Pose2d p = spline.getPose2d(j);
                double x = p.getX();
                double y = p.getY();
                Rotation2d heading = p.getRotation();
                double dx = m_scale * heading.getCos();
                double dy = m_scale * heading.getSin();
                s.add(x, y, dx, dy);
            }

        }
        return s;
    }

}
