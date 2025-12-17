package org.team100.lib.trajectory;

import org.jfree.data.xy.VectorSeries;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryToVectorSeries {

    /** Length of the vector indicating heading */
    private final double m_scale;

    public TrajectoryToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public VectorSeries convert(String name, Trajectory100 t) {
        VectorSeries s = new VectorSeries(name);
        double duration = t.duration();
        double dt = duration/20;
        for (double time = 0; time < duration; time += dt) {
            TimedPose p = t.sample(time);
            WaypointSE2 pp = p.state().getPose();
            double x = pp.pose().getTranslation().getX();
            double y = pp.pose().getTranslation().getY();
            Rotation2d heading = pp.pose().getRotation();
            double dx = m_scale * heading.getCos();
            double dy = m_scale * heading.getSin();
            s.add(x, y, dx, dy);
        }
        return s;
    }
}
